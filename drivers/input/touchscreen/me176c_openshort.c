/* drivers/input/touchscreen/gt9xx_shorttp.c
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:1.0
 * Author: meta@goodix.com
 * Accomplished Date:2012/10/20
 * Revision record:
 *
 */
#define OPENSHORT
#include "me176c_openshort.h"

// short test
#define GTP_SHORT_GND
#define GTP_VDD         33      // 3.3V

u32 opentestflag = 0;
u32 shorttestflag = 0;

// open test
u16 max_limit_value_Jtouch = 3060;     // screen max limit for Jtouch
u16 min_limit_value_Jtouch = 1610;     // screen min limit fot Jtouch
u16 max_limit_value_Ofilm = 3272;     // screen max limit for Ofilm
u16 min_limit_value_Ofilm = 1500;     // screen min limit fot Ofilm
/*
u16 max_limit_value_Wintek= 2500;            // screen max limit for Wintek
u16 min_limit_value_Wintek= 1506;            // screen min limit for Wintek
u16 max_limit_value_Innolux= 1480;      // screen max limit for Innolux
u16 min_limit_value_Innolux = 835;      // screen min limit for Innolux
*/
u16 max_limit_value = 1487;            // screen max limit
u16 min_limit_value = 843;            // screen min limit
u16 max_limit_key = 1631;              // key_val max limit
u16 min_limit_key = 625;               // key_val min limit
extern s32 gtp_i2c_read(struct i2c_client *, u8 *, s32);
extern s32 gtp_i2c_write(struct i2c_client *, u8 *, s32);
extern s8 gtp_i2c_test(struct i2c_client *client);
extern void gtp_reset_guitar(struct i2c_client*, s32);
extern s32 gup_enter_update_mode(struct i2c_client *);
extern s32 gup_leave_update_mode(void);
extern s32 gtp_send_cfg(struct i2c_client *client);
extern s32 gtp_read_version(struct i2c_client *client, u16* version);
extern void gtp_irq_disable(struct goodix_ts_data *ts);
extern void gtp_irq_enable(struct goodix_ts_data *ts);
extern s32 gup_i2c_write(struct i2c_client *client,u8 *buf,s32 len);
#if AREA_ACCORD_CHECK
extern void AreaAccordCheck(u16 *CurrentDataTemp);
#endif
#if ALL_ACCORD_CHECK
extern void AllAccordCheck(u16 *CurrentDataTemp);
#endif

u8 gtp_rawdiff_mode;
extern struct i2c_client * i2c_connect_client;
extern u8 config[GTP_ADDR_LENGTH + GTP_CONFIG_MAX_LENGTH];
#if AREA_ACCORD_CHECK
extern u32 accord_limit_Jtouch;
extern u32 accord_limit_Ofilm;
/*
extern u32 accord_limit_Wintek;
extern u32 accord_limit_Innolux;
*/
extern u32 accord_limit;
extern u32 AreaCheckResult;
extern u16 channel_status[MAX_SENSOR_NUM * MAX_DRIVER_NUM];
extern u16 beyond_accord_limit_num[MAX_SENSOR_NUM * MAX_DRIVER_NUM];
extern u32 beyond_accord_limit_val[MAX_SENSOR_NUM * MAX_DRIVER_NUM];
extern u8  special_case_limit;
#endif
#if ALL_ACCORD_CHECK
extern u32 all_accord_limit_Jtouch;
extern u32 all_accord_limit_Ofilm;
/*
extern u32 all_accord_limit_Innolux;
extern u32 all_accord_limit_Wintek;
*/
extern u32 all_accord_limit;
extern u32 AllCheckResult;
extern u16 all_channel_status[MAX_SENSOR_NUM * MAX_DRIVER_NUM];
extern u16 beyond_all_accord_limit_num[MAX_SENSOR_NUM * MAX_DRIVER_NUM];
extern u32 beyond_all_accord_limit_val[MAX_SENSOR_NUM * MAX_DRIVER_NUM];
extern u32 average;
#endif

u8  gt9xx_drv_num = MAX_DRIVER_NUM; // default driver and sensor number
u8  gt9xx_sen_num = MAX_SENSOR_NUM;
u16 gt9xx_pixel_cnt = MAX_DRIVER_NUM * MAX_SENSOR_NUM;
u16 gt9xx_sc_pxl_cnt = MAX_DRIVER_NUM * MAX_SENSOR_NUM;
struct gt9xx_short_info *short_sum; 

#if GTP_HAVE_TOUCH_KEY
    u8 gt9xx_sc_drv_num;
    u8 key_is_isolated; // 0: no, 1: yes
    u8 key_iso_pos[5];
#endif



u8 gt900_short_threshold = 10;
u8 gt900_resistor_threshold = 1000;
u8 gt900_resistor_warn_threshold = 500;
u8 gt900_gnd_resistor_threshold = 400;
u8 gt900_adc_read_delay = 150;
u8 gt900_diffcode_short_threshold = 20;

//QFN88 package TX Pin, used for get channel number from Chip to Pad
u8 ChannelPackage_TX[MAX_DRIVER_NUM] =  { 
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,
    20,21,22,23,24,25,/*26,*/27,28,29,30,31,32,33,34,35,36,37,
    38,39,40,41,42
};	

struct kobject *goodix_debug_kobj = NULL;
static s32 sample_set_num = 14; 
static u32 default_test_types = _MAX_TEST | _MIN_TEST | _KEY_MAX_TEST | _KEY_MIN_TEST | _AREA_TEST | _ALL_TEST;
static u8  rslt_buf_idx = 0;
static s32 *test_rslt_buf;  
static struct gt9xx_open_info *touchpad_sum;

#define _MIN_ERROR_NUM      (sample_set_num * 9 / 10)
                    
//static char *result_lines[32 * 24 + 20]; // [200];
static char *result_lines[31 * 17 +20];//for 176
static char tmp_info_line[80];
static u32 RsltIndex = 0;

static void append_info_line(void)
{
    if (strlen(tmp_info_line) != 0)
    {
        result_lines[RsltIndex] = (char *)kzalloc(strlen(tmp_info_line), GFP_KERNEL);
        memcpy(result_lines[RsltIndex], tmp_info_line, strlen(tmp_info_line));
    }
//    if (RsltIndex != (32 * 24 + 19)) //199)
       if (RsltIndex != (31 * 17 + 19))
        ++RsltIndex;
    else {
        kfree(result_lines[RsltIndex]);
    }
}


                  
#define SET_INFO_LINE_INFO(fmt, args...)       do{ memset(tmp_info_line, '\0', 80);\
                                                   sprintf(tmp_info_line, "<Sysfs-INFO>"fmt"\n", ##args);\
                                                   GTP_INFO(fmt, ##args);\
                                                append_info_line();} while(0)

#define SET_INFO_LINE_DEBUG(fmt, args...)       do{ memset(tmp_info_line, '\0', 80);\
                                                   sprintf(tmp_info_line, "<Sysfs-INFO>"fmt"\n", ##args);\
                                                   GTP_DEBUG(fmt, ##args);\
                                                append_info_line();} while(0)

#define SET_INFO_LINE_ERR(fmt, args...)        do { memset(tmp_info_line, '\0', 80);\
                                                   sprintf(tmp_info_line, "<Sysfs-ERROR>"fmt"\n", ##args);\
                                                   GTP_ERROR(fmt, ##args);\
                                                   append_info_line();}while(0)


static u8 cfg_drv_order[MAX_DRIVER_NUM];
static u8 cfg_sen_order[MAX_SENSOR_NUM];

/*
 * Initialize cfg_drv_order and cfg_sen_order, which is used for report short channels
 *
 */

s32 gt9xx_short_parse_cfg(void)
{
    u8 i = 0;
	u8 drv_num = 0, sen_num = 0;
	
	u8 config[256] = {(u8)(GTP_REG_CONFIG_DATA >> 8), (u8)GTP_REG_CONFIG_DATA, 0};
	
	if (gtp_i2c_read(i2c_connect_client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH) <= 0)
	{
	    SET_INFO_LINE_ERR("Failed to read config!");
	    return FAIL;
	}
	
	drv_num = (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT-GT9_REG_CFG_BEG] & 0x1F)
						+ (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+1 -GT9_REG_CFG_BEG] & 0x1F);
	sen_num = (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG] & 0x0F) 
						+ ((config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG]>>4) & 0x0F);

	if (drv_num < MIN_DRIVER_NUM || drv_num > MAX_DRIVER_NUM)
	{
		GTP_ERROR("driver number error!");
		return FAIL;
	}
	if (sen_num < MIN_SENSOR_NUM || sen_num > MAX_SENSOR_NUM)
	{
		GTP_ERROR("sensor number error!");
		return FAIL;
	}
	// get sensor and driver order 
	memset(cfg_sen_order, 0xFF, MAX_SENSOR_NUM);
    for (i = 0; i < sen_num; ++i)
	{
	    cfg_sen_order[i] = config[GTP_ADDR_LENGTH + GT9_REG_SEN_ORD - GT9_REG_CFG_BEG + i];
	}

	memset(cfg_drv_order, 0xFF, MAX_DRIVER_NUM);
	for (i = 0; i < drv_num; ++i)
	{
	    cfg_drv_order[i] = config[GTP_ADDR_LENGTH + GT9_REG_DRV_ORD - GT9_REG_CFG_BEG + i];
	}
    
    return SUCCESS;
}

/*
 * @param:
 *      phy_chnl: ic detected short channel, is_driver: it's driver or not
 * @Return:
 *      0xff: the ic channel is not used, otherwise: the tp short channel
 */
u8 gt9_get_short_tp_chnl(u8 phy_chnl, u8 is_driver)
{
    u8 i = 0;
    if (is_driver) {
        for (i = 0; i < MAX_DRIVER_NUM; ++i)
        {
            if (cfg_drv_order[i] == phy_chnl) {
                return i;
            }
            else if (cfg_drv_order[i] == 0xff) {
                return 0xff;
            }
        }
    }
    else 
    {
        for (i = 0; i < MAX_SENSOR_NUM; ++i)
        {
            if (cfg_sen_order[i] == phy_chnl) {
                return i;
            }
            else if (cfg_sen_order[i] == 0xff) {
                return 0xff;
            }
        }
    }
    return 0xff;
}


u8 gt9xx_set_ic_msg(struct i2c_client *client, u16 addr, u8 val)
{
    s32 i = 0;
    u8 msg[3];

    msg[0] = (addr >> 8) & 0xff;
    msg[1] = addr & 0xff;
    msg[2] = val;

    for (i = 0; i < 5; i++)
    {
        if (gtp_i2c_write(client, msg, GTP_ADDR_LENGTH + 1) > 0)
        {
            break;
        }
    }

    if (i >= 5)
    {
        GTP_ERROR("Set data to 0x%02x%02x failed!", msg[0], msg[1]);
        return FAIL;
    }

    return SUCCESS;
}

static s32 gtp_i2c_end_cmd(struct i2c_client *client)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    s32 ret = 0;
    
    ret = gtp_i2c_write(client, end_cmd, 3);
    if (ret < 0)
    {
        SET_INFO_LINE_INFO("I2C write end_cmd  error!"); 
    }
    return ret;
}

s32 gtp_parse_config(void)
{
#if GTP_HAVE_TOUCH_KEY
    u8 i = 0;
    u8 key_pos = 0;
#endif
    int j = 0;
    struct goodix_ts_data *ts;
    u8 chksum = 0;
    u8 config[256] = {(u8)(GTP_REG_CONFIG_DATA >> 8), (u8)GTP_REG_CONFIG_DATA, 0};
	
	if (gtp_i2c_read(i2c_connect_client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH) <= 0)
	{
	    SET_INFO_LINE_ERR("Failed to read config!");
	    return FAIL;
	}
    // disable hopping
    if (config[GTP_ADDR_LENGTH + 0x807D - GTP_REG_CONFIG_DATA] & 0x80)
    {
        config[GTP_ADDR_LENGTH + 0x807D - GTP_REG_CONFIG_DATA] &= 0x7F;
        ts = i2c_get_clientdata(i2c_connect_client);
        // calculate checksum
        for (j = 0; j < (ts->gtp_cfg_len-2); ++j)
        {
            chksum += config[GTP_ADDR_LENGTH + j];
        }
        config[ts->gtp_cfg_len] = (~chksum) + 1;
        
        gup_i2c_write(i2c_connect_client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
    }
    gt9xx_drv_num = (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT-GT9_REG_CFG_BEG] & 0x1F)
                    + (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+1 -GT9_REG_CFG_BEG] & 0x1F);
    gt9xx_sen_num = (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG] & 0x0F) 
                    + ((config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG]>>4) & 0x0F);

    if (gt9xx_drv_num < MIN_DRIVER_NUM || gt9xx_drv_num > MAX_DRIVER_NUM)
    {
        SET_INFO_LINE_ERR("driver number error!");
        return FAIL;
    }
    if (gt9xx_sen_num < MIN_SENSOR_NUM || gt9xx_sen_num > MAX_SENSOR_NUM)
    {
        SET_INFO_LINE_ERR("sensor number error!");
        return FAIL;
    }
    gt9xx_sc_pxl_cnt = gt9xx_pixel_cnt = gt9xx_drv_num * gt9xx_sen_num;
    
#if GTP_HAVE_TOUCH_KEY
    gt9xx_sc_drv_num = gt9xx_drv_num - (config[0x804E - GT9_REG_CFG_BEG + GTP_ADDR_LENGTH] & 0x01);
    
    key_is_isolated = 0;
    key_iso_pos[0] = 0;
    for (i = 0; i < 4; ++i)
    {
        key_pos = config[GTP_ADDR_LENGTH + GT9_REG_KEY_VAL - GT9_REG_CFG_BEG + i]%0x08;
        GTP_DEBUG("key_val[%d] = 0x%x", i+1, config[GTP_ADDR_LENGTH + GT9_REG_KEY_VAL - GT9_REG_CFG_BEG + i]);
        if (key_pos != 0)
        {
            break;
        }
        else
        {
            key_iso_pos[0]++;
            key_iso_pos[i+1] = config[GTP_ADDR_LENGTH + GT9_REG_KEY_VAL - GT9_REG_CFG_BEG + i]/8;
        }
    }
    if (i == 4)
    {
        key_is_isolated = 1;
    }
    gt9xx_sc_pxl_cnt = gt9xx_pixel_cnt - (gt9xx_drv_num-gt9xx_sc_drv_num) * gt9xx_sen_num;
    GTP_DEBUG("drv num: %d, sen num: %d, sc drv num: %d", gt9xx_drv_num, gt9xx_sen_num, gt9xx_sc_drv_num);
    if (key_is_isolated)
    {
        GTP_DEBUG("[%d key(s)]: %d, %d, %d, %d", key_iso_pos[0], key_iso_pos[1], key_iso_pos[2], key_iso_pos[3], key_iso_pos[4]);
    }
#endif

    return SUCCESS;
}



/*
 * Function:
 * 		write one byte to specified register
 * Input:
 * 		reg: the register address
 * 		val: the value to write into
 * Return:
 * 		i2c_write function return 
 */
s32 gtp_write_register(struct i2c_client * client, u16 reg, u8 val)
{
    u8 buf[3];
    buf[0] = (u8) (reg >> 8);
    buf[1] = (u8) reg;
    buf[2] = val;
    return gtp_i2c_write(client, buf, 3);
}
/*
 * Function: 
 * 		read one byte from specified register into buf
 * Input:
 *		reg: the register
 * 		buf: the buffer for one byte
 * Return:
 *		i2c_read function return
 */
s32 gtp_read_register(struct i2c_client * client, u16 reg, u8* buf)
{
    buf[0] = (u8)(reg >> 8);
    buf[1] = (u8)reg;
    return gtp_i2c_read(client, buf, 3);
}

/* 
 * Function:
 * 		burn dsp_short code
 * Input:
 * 		i2c_client
 * Return:
 * 		SUCCESS: burning succeed, FAIL: burning failed
 */
s32 gtp_burn_dsp_short(struct i2c_client *client)
{
    s32 ret = 0;
    u8 *opr_buf;
    u16 i = 0;
    u16 addr = GTP_REG_DSP_SHORT;
    u16 opr_len = 0;
    u16 left = 0;

    GTP_DEBUG("Start writing dsp_short code");
    opr_buf = (u8*)kmalloc(sizeof(u8) * (2048+2), GFP_KERNEL);
    if (!opr_buf)
    {
        SET_INFO_LINE_ERR("failed to allocate memory for check buffer!");
        return FAIL;
    }
    
    left = sizeof(dsp_short);
    while (left > 0)
    {
        opr_buf[0] = (u8)(addr >> 8);
        opr_buf[1] = (u8)(addr);
        if (left > 2048)
        {
            opr_len = 2048;
        }
        else
        {
            opr_len = left;
        }
        memcpy(&opr_buf[2], &dsp_short[addr-GTP_REG_DSP_SHORT], opr_len);
        ret = gtp_i2c_write(client, opr_buf, 2 + opr_len);
        if ( ret < 0 )
        {
            SET_INFO_LINE_ERR("write dsp_short code failed!");
            kfree(opr_buf);
            return FAIL;
        }
        addr += opr_len;
        left -= opr_len;
    }
    
    // check code: 0xC000~0xCFFF
    GTP_DEBUG("Start checking dsp_short code");
    addr = GTP_REG_DSP_SHORT;
    left = sizeof(dsp_short);
    while (left > 0)
    {
        opr_buf[0] = (u8)(addr >> 8);
        opr_buf[1] = (u8)(addr);
        
        msleep(20);
        
        if (left > 2048)
        {
            opr_len = 2048;
        }
        else
        {
            opr_len = left;
        }
        
        ret = gtp_i2c_read(client, opr_buf, opr_len+2);
        if (ret < 0)
        {
            kfree(opr_buf);
            return FAIL;
        }
        for (i = 0; i < opr_len; ++i)
        {
            if (opr_buf[i+2] != dsp_short[addr-GTP_REG_DSP_SHORT+i])
            {
                SET_INFO_LINE_ERR("check dsp_short code failed!");
                kfree(opr_buf);
                return FAIL;
            }
        }
        
        addr += opr_len;
        left -= opr_len;
    }
    kfree(opr_buf);
    return SUCCESS;
}
/*
 * Function: 
 * 		check the resistor between shortlike channels if less than threshold confirm as short
 * INPUT:
 *		Short like Information struct pointer
 * Returns:
 *		SUCCESS: it's shorted FAIL: otherwise
 */
s32 gtp_short_resist_check(struct gt9xx_short_info *short_node)
{
    s32 short_resist = 0;
    struct gt9xx_short_info *node = short_node;
    u8 master = node->master;
    u8 slave = node->slave;
    u8 chnnl_tx[4] = { GT9_DRV_HEAD|13, GT9_DRV_HEAD|28,
                    GT9_DRV_HEAD|29, GT9_DRV_HEAD|42 };
    s32 numberator = 0;
    u32 amplifier = 1000;  // amplify 1000 times to emulate float computing
    
    
    // Tx-ABIST & Tx_ABIST
    if ((((master > chnnl_tx[0]) && (master <= chnnl_tx[1])) &&
        ((slave > chnnl_tx[0]) && (slave <= chnnl_tx[1])) ) ||
        (((master > chnnl_tx[2]) && (master <= chnnl_tx[3])) &&
        ((slave > chnnl_tx[2]) && (master <= chnnl_tx[3]))))
    {
        numberator = node->self_data * 40 * amplifier;
        short_resist = numberator/(node->short_code) - 40 * amplifier;
    }
    // Receiver is Rx-odd(1,3,5)
    else if ((node->slave & (GT9_DRV_HEAD | 0x01)) == 0x01)
    {
        numberator = node->self_data * 60 * amplifier;
        short_resist = numberator/node->short_code - 40 * amplifier; 
    }
    else
    {
        numberator = node->self_data * 60 * amplifier;
        short_resist = numberator / node->short_code - 60 * amplifier;
    }
    GTP_DEBUG("self_data = %d" ,node->self_data);
    GTP_DEBUG("master = 0x%x, slave = 0x%x", node->master, node->slave);
    GTP_DEBUG("short_code = %d, short_resist = %d", node->short_code, short_resist);
    
    if (short_resist < 0)
    {
        short_resist = 0;
    }
    
    if (short_resist < (gt900_resistor_threshold * amplifier))
    {
        node->impedance = short_resist / amplifier;
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}



/*
 * Function: 
 * 		compute the result, whether there are shorts or not
 * Input:
 * 		i2c_client
 * Return:
 * 		SUCCESS
 */
s32 gtp_compute_rslt(struct i2c_client *client)
{
    u16 short_code;
    u8 i = 0, j = 0;
    u16 result_addr;
    u8 *result_buf;
    u16 *self_data;
    s32 ret = 0;
    u16 data_len = 3 + (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2 + 2; // a short data frame length
    struct gt9xx_short_info short_node;
    u16 node_idx = 0; // short_sum index: 0~GT9_INFO_NODE_MAX
    
    u8 tx_short_num = 0;
    u8 rx_short_num = 0;
    
    u8 master, slave;
    
    self_data = (u16*)kmalloc(sizeof(u16) * ((MAX_DRIVER_NUM + MAX_SENSOR_NUM)), GFP_KERNEL);
    result_buf = (u8*)kmalloc(sizeof(u8) * (data_len+2), GFP_KERNEL);
    short_sum = (struct gt9xx_short_info *) kmalloc(sizeof(struct gt9xx_short_info) * GT9_INFO_NODE_MAX, GFP_KERNEL);

    if (!self_data || !result_buf || !short_sum)
    {
        SET_INFO_LINE_ERR("allocate memory for short result failed!");
        return FAIL;
    }	
    
    // Get Selfdata
    result_buf[0] = 0xA4;
    result_buf[1] = 0xA1;
    gtp_i2c_read(client, result_buf, 2 + 144);
    for (i = 0, j = 0; i < 144; i += 2)
    {
        self_data[j++] = (u16)(result_buf[i] << 8) + (u16)(result_buf[i+1]);
    }
    GTP_DEBUG("Self Data:");
    GTP_DEBUG_ARRAY(result_buf+2, 144);
    
    
    // Get TxShortNum & RxShortNum
    result_buf[0] = 0x88;
    result_buf[1] = 0x02;
    gtp_i2c_read(client, result_buf, 2 + 2);
    tx_short_num = result_buf[2];
    rx_short_num = result_buf[3];
    
    GTP_DEBUG("Tx Short Num: %d, Rx Short Num: %d", tx_short_num, rx_short_num);
    
    // 
    result_addr = 0x8860;
    data_len = 3 + (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2 + 2;
    for (i = 0; i < tx_short_num; ++i)
    {        
        result_buf[0] = (u8) (result_addr >> 8);
        result_buf[1] = (u8) (result_addr);
        ret = gtp_i2c_read(client, result_buf, data_len+2);
        if (ret < 0)
        {
            SET_INFO_LINE_ERR("read result data failed!");
        }
        GTP_DEBUG("Result Buffer: ");
        GTP_DEBUG_ARRAY(result_buf+2, data_len);
        
        short_node.master_is_driver = 1;
        short_node.master = result_buf[2];
        
        // Tx - Tx
        for (j = i + 1; j < MAX_DRIVER_NUM; ++j)
        {
            short_code = (result_buf[2+3+j*2] << 8) + result_buf[2+3+j*2+1];
            if (short_code > gt900_short_threshold)
            {
                short_node.slave_is_driver = 1;
                short_node.slave = ChannelPackage_TX[j] | GT9_DRV_HEAD;
                short_node.self_data = self_data[j];
                short_node.short_code = short_code;
                
                ret = gtp_short_resist_check(&short_node);	
                if (ret == SUCCESS)
                {
                    if (node_idx < GT9_INFO_NODE_MAX)
                    {
                        short_sum[node_idx++] = short_node;
                    }
                }
            }
        }
        // Tx - Rx
        for (j = 0; j < MAX_SENSOR_NUM; ++j)
        {
            short_code = (result_buf[2+3+84+j*2] << 8) + result_buf[2+3+84+j*2+1];
            
            if (short_code > gt900_short_threshold)
            {
                short_node.slave_is_driver = 0;
                short_node.slave = j | GT9_SEN_HEAD;
                short_node.self_data = self_data[MAX_DRIVER_NUM + j];
                short_node.short_code = short_code;
                
                ret = gtp_short_resist_check(&short_node);
                if (ret == SUCCESS)
                {
                    if (node_idx < GT9_INFO_NODE_MAX)
                    {
                        short_sum[node_idx++] = short_node;
                    }
                }
            }
        }
        
        result_addr += data_len;
    }
    
    result_addr = 0xA0D2;
    data_len = 3 + MAX_SENSOR_NUM * 2 + 2;
    for (i = 0; i < rx_short_num; ++i)
    {
        result_buf[0] = (u8) (result_addr >> 8);
        result_buf[1] = (u8) (result_addr);
        ret = gtp_i2c_read(client, result_buf, data_len + 2);
        if (ret < 0)
        {
            SET_INFO_LINE_ERR("read result data failed!");
        }
        
        GTP_DEBUG("Result Buffer: ");
        GTP_DEBUG_ARRAY(result_buf+2, data_len);
        
        short_node.master_is_driver = 0;
        short_node.master = result_buf[2];
        
        // Rx - Rx
        for (j = 0; j < MAX_SENSOR_NUM; ++j)
        {
            if ((j == i) || ( (j < i) && (j & 0x01) == 0))
            {
                continue;
            }
            short_code = (result_buf[2+3+j*2] << 8) + result_buf[2+3+j*2+1];
            
            if (short_code > gt900_short_threshold)
            {
                short_node.slave_is_driver = 0;
                short_node.slave = j | GT9_SEN_HEAD;
                short_node.self_data = self_data[MAX_DRIVER_NUM + j];
                short_node.short_code = short_code;
                
                ret = gtp_short_resist_check(&short_node);
                if (ret == SUCCESS)
                {
                    if (node_idx < GT9_INFO_NODE_MAX)
                    {
                        short_sum[node_idx++] = short_node;
                    }
                }
            }
        }
        
        result_addr += data_len;
    }
    
    if (node_idx == 0)
    {
        ret = SUCCESS;
    }
    else
    {
        for (i = 0, j = 0; i < node_idx; ++i)
        {
            if ((short_sum[i].master_is_driver))
            {
                if (short_sum[i].master > (26 | GT9_DRV_HEAD))
                {
                    short_sum[i].master--;
                }
                master = gt9_get_short_tp_chnl(short_sum[i].master-GT9_DRV_HEAD, 1);
                
            }
            else
            {
                master = gt9_get_short_tp_chnl(short_sum[i].master, 0);
            }
            
            if ((short_sum[i].slave_is_driver))
            {
                if (short_sum[i].slave > (26 | GT9_DRV_HEAD))
                {
                    short_sum[i].slave--;
                }
                slave = gt9_get_short_tp_chnl(short_sum[i].slave-GT9_DRV_HEAD, 1);
             
            }
            else
            {
                slave = gt9_get_short_tp_chnl(short_sum[i].slave, 0);
            }
            GTP_DEBUG("Orignal Shorted Channels: %s%d, %s%d",
                (short_sum[i].master_is_driver) ? "Drv" : "Sen", master,
                (short_sum[i].slave_is_driver) ? "Drv" : "Sen", slave);
            
            if (master == 255 && slave == 255)
            {
                GTP_DEBUG("unbonded channel (%d, %d) shorted!", short_sum[i].master, short_sum[i].slave);
                continue;
            }
            else
            {
                short_sum[j].slave = slave;
                short_sum[j].master = master;
                short_sum[j].slave_is_driver = short_sum[i].slave_is_driver;
                short_sum[j].master_is_driver = short_sum[i].master_is_driver;
                short_sum[j].impedance = short_sum[i].impedance;
                short_sum[j].self_data = short_sum[i].self_data;
                short_sum[j].short_code = short_sum[i].short_code;
                ++j;
            }
        }
        node_idx = j;
        if (node_idx == 0)
        {
            ret = SUCCESS;
        }
        else
        {
            for (i = 0; i < node_idx; ++i)
            {
                SET_INFO_LINE_INFO("  %s%02d & %s%02d Shorted! (R = %dKOhm)",
                (short_sum[i].master_is_driver) ? "Drv" : "Sen", short_sum[i].master,
                (short_sum[i].slave_is_driver) ? "Drv" : "Sen", short_sum[i].slave,
                short_sum[i].impedance);
            }
            ret = FAIL;
        }
    }
    kfree(self_data);
    kfree(short_sum);
    kfree(result_buf);
    return ret;
}

s32 gt9_test_gnd_vdd_short(struct i2c_client *client)
{
    
    u8 *data;
    s32 ret = 0;
    s32 i = 0;
    u16 len = (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2;
    u16 short_code = 0;
    s32 r = -1;
    u32 short_res = 0;
    u16 amplifier = 1000;
    
    data = (u8 *)kmalloc(sizeof(u8) * (len + 2), GFP_KERNEL);
    if (NULL == data)
    {
       SET_INFO_LINE_ERR("failed to allocate memory for gnd vdd test data buffer");
       return FAIL;
    }
    
    data[0] = 0xA5;
    data[1] = 0x31;
    gtp_i2c_read(client, data, 2 + len);
    
    GTP_DEBUG_ARRAY(data+2, len);
    ret = SUCCESS;
    for (i = 0; i < len; i += 2)
    {
        short_code = (data[2+i] << 8) + (data[2 + i + 1]);
        if (short_code == 0)
        {
            continue;
        }
        if ((short_code & 0x8000) == 0)        // short with GND
        {
        #ifdef GTP_SHORT_GND
            r = 5266285 * 10 / (short_code & (~0x8000)) - 40 * amplifier;
        #endif
        }
        else    // short with VDD
        {
            //r = ( 1/(((float)(short_code&(~0x8000)))/0.9*0.7/1024/(sys.avdd-0.9)/40) ) -40;
        #ifdef GTP_VDD
            r = 40*9*1024*(100*GTP_VDD - 900)/((short_code&(~0x8000))*7) - 40*1000;
            GTP_DEBUG("vdd short_code: %d", short_code & (~0x8000));
        #endif
        }
        GTP_DEBUG("resistor: %d, short_code: %d", r, short_code);
        
        short_res = (r >= 0) ? r : 0xFFFF;
        if (short_res == 0xFFFF)
        {
        }
        else 
        {
            if (short_res < (gt900_gnd_resistor_threshold * amplifier))
            {
                if (i < MAX_DRIVER_NUM * 2)       // driver 
                {
                    SET_INFO_LINE_INFO("  Drv%02d & GND/VDD Shorted! (R = %dKOhm)", ChannelPackage_TX[i/2], short_res/amplifier);
                }
                else
                {
                    SET_INFO_LINE_INFO("  Sen%02d & GND/VDD Shorted! (R = %dKOhm)", (i/2) - MAX_DRIVER_NUM, short_res/amplifier);
                }
                ret = FAIL;
            }
        }
    }
    return ret;
}


/*
 * leave short test 
 */
void gt9xx_leave_short_test(struct i2c_client *client)
{
    // boot from rom and download code from flash to ram
    gtp_write_register(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
    gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x08);
    
    gtp_reset_guitar(client, 20);
    msleep(100);

    gtp_send_cfg(client);
    SET_INFO_LINE_INFO("");
    SET_INFO_LINE_INFO("---gtp short test end---");
}


/*
 * Function:
 *		gt9 series ic short test function
 * Input:
 * 		I2c_client, i2c device
 * Return:
 * 		SUCCESS: test succeed, FAIL: test failed
 */
s32 gt9xx_short_test(struct i2c_client * client)
{
    s32 ret = 0;
    s32 ret2 = 0;
    u8 i = 0;
    u8 opr_buf[60] = {0};
    u8 retry = 0;
    u8 drv_sen_chksum = 0;
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(i2c_connect_client);
    //gtp_irq_disable(ts);
    disable_irq(ts->client->irq);
#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 1;     // suspend esd
#endif
    // step 1: reset guitar, delay 1ms,  hang up ss51 and dsp
    SET_INFO_LINE_INFO("---gtp short test---");
    SET_INFO_LINE_INFO("Step 1: reset guitar, hang up ss51 dsp");

    if (gtp_i2c_test(client) < 0)
    {
        SET_INFO_LINE_ERR("I2C test failed!");
        goto short_test_exit;
    }
    
    gt9xx_short_parse_cfg();
    
    // RST output low last at least 2ms
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    msleep(2);
    
    // select I2C slave addr,INT:0--0xBA;1--0x28.
    GTP_GPIO_OUTPUT(GTP_INT_PORT, (client->addr == 0x14));
    msleep(2);
    
    // RST output high reset guitar
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    
    while(retry++ < 200)
    {
        // Hold ss51 & dsp
        ret = gtp_write_register(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
        if(ret <= 0)
        {
            GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
            gtp_reset_guitar(client, 10);
            continue;
        }
        
        // Confirm hold
        ret = gtp_read_register(client, _rRW_MISCTL__SWRST_B0_, opr_buf);
        if(ret <= 0)
        {
            GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
            gtp_reset_guitar(client, 10);
            continue;
        }
        if(0x0C == opr_buf[GTP_ADDR_LENGTH])
        {
            GTP_DEBUG("Hold ss51 & dsp confirm SUCCESS");
            break;
        }
        GTP_DEBUG("Hold ss51 & dsp confirm 0x4180 failed,value:%d", opr_buf[GTP_ADDR_LENGTH]);
    }
    if(retry >= 200)
    {
        GTP_ERROR("Enter update Hold ss51 failed.");
        return FAIL;
    }
    // DSP_CK and DSP_ALU_CK PowerOn
    gtp_write_register(client, 0x4010, 0x00);
	SET_INFO_LINE_INFO("Enter short test mode SUCCESS."); 

    // step2: burn dsp_short code
    SET_INFO_LINE_INFO("step 2: burn dsp_short code");
    gtp_write_register(client, _bRW_MISCTL__TMR0_EN, 0x00); // clear watchdog
    gtp_write_register(client, _bRW_MISCTL__CACHE_EN, 0x00); // clear cache
    gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x02); // boot from sram
    gtp_write_register(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01); // reset software
    gtp_write_register(client, _bRW_MISCTL__SRAM_BANK, 0x00); // select bank 0
    gtp_write_register(client, _bRW_MISCTL__MEM_CD_EN, 0x01); // allow AHB bus accessing code sram
    
    // ---: burn dsp_short code
    ret = gtp_burn_dsp_short(client);
    if (ret == FAIL)
    {
        SET_INFO_LINE_ERR("step2: burn dsp_short failed!");
        goto short_test_exit;
    }
    
    // step3: run dsp_short, read results
    SET_INFO_LINE_INFO("step 3: run dsp_short code, confirm it's runnin'");
    gtp_write_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, 0x00);	// clear dsp_short running flag
    gtp_write_register(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x03);//set scramble

	ret = gt9xx_set_ic_msg(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01);           //20121114
    gtp_write_register(client, _rRW_MISCTL__SWRST_B0_, 0x08);	// release dsp
    
    msleep(80);
    // confirm dsp is running
    i = 0;
    while (1)
    {
        opr_buf[2] = 0x00;
        gtp_read_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, opr_buf);
        if (opr_buf[2] == 0xAA)
        {
            break;
        }
        ++i;
        if (i >= 8)
        {
            SET_INFO_LINE_ERR("step 3: dsp is not running!");
            goto short_test_exit;
        }
        msleep(10);
    }
    // step4: host configure ic, get test result
    SET_INFO_LINE_INFO("Step 4: host config ic, get test result");
    // Short Threshold
    GTP_DEBUG(" Short Threshold: %d", gt900_short_threshold);
    opr_buf[0] = (u8) (GTP_REG_SHORT_TH >> 8);
    opr_buf[1] = (u8) GTP_REG_SHORT_TH;
    opr_buf[2] = (u8)(gt900_short_threshold >> 8);
    opr_buf[3] = (u8)(gt900_short_threshold & 0xFF);
    gtp_i2c_write(client, opr_buf, 4);
    
    // ADC Read Delay
    GTP_DEBUG(" ADC Read Delay: %d", gt900_adc_read_delay);
    opr_buf[1] += 2;
    opr_buf[2] = (u8)(gt900_adc_read_delay >> 8);
    opr_buf[3] = (u8)(gt900_adc_read_delay & 0xFF);
    gtp_i2c_write(client, opr_buf, 4);
    
    // DiffCode Short Threshold
    GTP_DEBUG(" DiffCode Short Threshold: %d", gt900_diffcode_short_threshold);
    opr_buf[0] = 0x88;
    opr_buf[1] = 0x51;
    opr_buf[2] = (u8)(gt900_diffcode_short_threshold >> 8);
    opr_buf[3] = (u8)(gt900_diffcode_short_threshold & 0xFF);
    gtp_i2c_write(client, opr_buf, 4);
    
    // Config Driver & Sensor Order
#if GTP_DEBUG_ON
    printk("<<-GTP-DEBUG->>: Driver Map:\n");
    printk("IC Driver:");
    for (i = 0; i < MAX_DRIVER_NUM; ++i)
    {
        printk(" %d", cfg_drv_order[i]);
    }
    printk("\n");
    printk("TP Driver:");
    for (i = 0; i < MAX_DRIVER_NUM; ++i)
    {
        printk(" %d", i);
    }
    printk("\n");
    
    printk("<<-GTP-DEBUG->>: Sensor Map:\n");
    printk("IC Sensor:");
    for (i = 0; i < MAX_SENSOR_NUM; ++i)
    {
        printk(" %d", cfg_sen_order[i]);
    }
    printk("\n");
    printk("TP Sensor:");
    for (i = 0; i < MAX_SENSOR_NUM; ++i)
    {
        printk(" %d", i);
    }
    printk("\n");
#endif

    opr_buf[0] = 0x88;
    opr_buf[1] = 0x08;
    for (i = 0; i < MAX_DRIVER_NUM; ++i)
    {
        opr_buf[2 + i] = cfg_drv_order[i];
        drv_sen_chksum += cfg_drv_order[i];
    }
    gtp_i2c_write(client, opr_buf, MAX_DRIVER_NUM + 2);
    
    opr_buf[0] = 0x88;
    opr_buf[1] = 0x32;
    for (i = 0; i < MAX_SENSOR_NUM; ++i)
    {
        opr_buf[2+i] = cfg_sen_order[i];
        drv_sen_chksum += cfg_sen_order[i];
    }
    gtp_i2c_write(client, opr_buf, MAX_SENSOR_NUM + 2);
    
    opr_buf[0] = 0x88;
    opr_buf[1] = 0x50;
    opr_buf[2] = drv_sen_chksum;
    gtp_i2c_write(client, opr_buf, 2 + 1);
    
    // clear waiting flag, run dsp
    gtp_write_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, 0x04);
    
    // inquirying test status until it's okay
    for (i = 0;;++i)
    {
        gtp_read_register(client, 0x8800, opr_buf);
        if (opr_buf[2] == 0x88)
        {
            break;
        }
        msleep(50);
        if ( i > 100 )
        {
            SET_INFO_LINE_ERR("step 4: inquiry test status timeout!");
            goto short_test_exit;
        }
    }
    
    // step 5: compute the result
    /* short flag: 
          bit0: Rx & Rx 
          bit1: Tx & Tx 
          bit2: Tx & Rx
          bit3: Tx/Rx & GND/VDD
    */
    gtp_read_register(client, 0x8801, opr_buf);
    GTP_DEBUG("short_flag = 0x%x", opr_buf[2]);
    SET_INFO_LINE_INFO("");
    SET_INFO_LINE_INFO("Short Test Result:");
    if ((opr_buf[2] & 0x0f) == 0)
    {
		shorttestflag =1;
        SET_INFO_LINE_INFO("  PASS!");
        ret = SUCCESS;
    }
    else 
    {
        if ((opr_buf[2] & 0x08) == 0x08)
        {
            ret2 = gt9_test_gnd_vdd_short(client);
            
        }
        ret = gtp_compute_rslt(client);
        if (ret == SUCCESS && ret2 == SUCCESS)
        {
			shorttestflag =1;
            SET_INFO_LINE_INFO("  PASS!");
        }
    }
    gt9xx_leave_short_test(client);
    //gtp_irq_enable(ts);
    enable_irq(ts->client->irq);

#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 0;     // resume esd
#endif
    return ret;
    
short_test_exit:
    gt9xx_leave_short_test(client);
    //gtp_irq_enable(ts);
    enable_irq(ts->client->irq);
#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 0;     // resume esd
#endif
    return FAIL;	
}

u32 endian_mode(void)
{
    union {s32 i; s8 c;}endian;

    endian.i = 1;

    if (1 == endian.c)
    {
        return MYBIG_ENDIAN;
    }
    else
    {
        return MYLITLE_ENDIAN;
    }
}
/*
*********************************************************************************************************
* Function: 
*	send read rawdata cmd
* Input:
*	i2c_client* client: i2c device
* Return:
* 	SUCCESS: send process succeed, FAIL: failed
*********************************************************************************************************
*/
s32 gt9_read_raw_cmd(struct i2c_client* client)
{
    u8 raw_cmd[3] = {(u8)(GTP_REG_READ_RAW >> 8), (u8)GTP_REG_READ_RAW, 0x01};
    s32 ret = -1;

    ret = gtp_i2c_write(client, raw_cmd, 3);
    if(ret <= 0)
    {
        SET_INFO_LINE_ERR("i2c write failed.");
        return FAIL;
    }
    msleep(10); 
    return SUCCESS;
}

s32 gt9_read_coor_cmd(struct i2c_client *client)
{
    u8 raw_cmd[3] = {(u8)(GTP_REG_READ_RAW >> 8), (u8)GTP_REG_READ_RAW, 0x0};
    s32 ret = -1;
    
    ret = gtp_i2c_write(client, raw_cmd, 3);
    if (ret < 0)
    {
        SET_INFO_LINE_ERR("i2c write coor cmd failed!");
        return FAIL;
    }
    msleep(10);
    return SUCCESS;
}
/*
*********************************************************************************************************
* Function: 
*	read rawdata from ic registers
* Input:
*	u16* data: rawdata buffer
* 	i2c_client* client: i2c device
* Return:
* 	SUCCESS: read process succeed, FAIL:  failed
*********************************************************************************************************
*/
s32 gtp_read_rawdata(struct i2c_client* client, u16* data)
{
    s32 ret = -1;
    u16 retry = 0;
    u8 read_state[3] = {(u8)(GTP_REG_RAW_READY>>8), (u8)GTP_REG_RAW_READY, 0};
    u16 i = 0, j = 0;
    u8 *read_rawbuf;
    u8 tail, head;

    read_rawbuf = (u8*)kmalloc(sizeof(u8) * (gt9xx_pixel_cnt * 2 + GTP_ADDR_LENGTH), GFP_KERNEL);

    if (NULL == read_rawbuf)
    {
        SET_INFO_LINE_ERR("failed to allocate for read_rawbuf");
        return FAIL;
    }
    read_rawbuf[0] = (u8)( GTP_REG_RAW_DATA >> 8);
    read_rawbuf[1] = (u8)( GTP_REG_RAW_DATA );
   
    if(data == NULL)
    {
        SET_INFO_LINE_ERR("Invalid raw buffer.");
        goto have_error;
    }
    
    msleep(10);
    while (retry++ < GTP_WAIT_RAW_MAX_TIMES)
    {
        ret = gtp_i2c_read(client, read_state, 3);
        if(ret <= 0)
        {
            SET_INFO_LINE_ERR("i2c read failed.return: %d", ret);
            continue;
        }
        if(read_state[GTP_ADDR_LENGTH] == 0x80)
        {
            GTP_DEBUG("Raw data is ready.");
            break;
        } 
        if ((retry/10) == 0)
        GTP_DEBUG("read_state[2] = 0x%x", read_state[GTP_ADDR_LENGTH]);
        msleep(5);
    }
    if (retry >= GTP_WAIT_RAW_MAX_TIMES)
    {
        SET_INFO_LINE_ERR("Wait raw data ready timeout.");
        goto have_error;
    }
    
    ret = gtp_i2c_read(client, read_rawbuf, GTP_ADDR_LENGTH + ((gt9xx_drv_num*gt9xx_sen_num)*2));
    if(ret <= 0)
    {
        SET_INFO_LINE_ERR("i2c read rawdata failed.");
        goto have_error;
    }
    gtp_i2c_end_cmd(client);	// clear buffer state

    if (endian_mode() == MYBIG_ENDIAN)
    {
        head = 0;
        tail =1;
        GTP_DEBUG("Big Endian.");
    }
    else
    {
        head = 1;
        tail = 0;
        GTP_DEBUG("Little Endian.");
    }
    
    for(i=0,j = 0; i < ((gt9xx_drv_num*gt9xx_sen_num)*2); i+=2)
    {
        data[i/2] = (u16)(read_rawbuf[i+head+GTP_ADDR_LENGTH]<<8) + (u16)read_rawbuf[GTP_ADDR_LENGTH+i+tail];
    #if GTP_DEBUG_ARRAY_ON
        printk("%d ", data[i/2]);
        ++j;
        if((j%10) == 0)
            printk("\n");
    #endif
    }
    
    kfree(read_rawbuf);
    return SUCCESS;
have_error:
	kfree(read_rawbuf);
	return FAIL;
}
/*
*********************************************************************************************************
* Function: 
*	rawdata test initilization function
* Input:
*	u32 check_types: test items
*********************************************************************************************************
*/
static void gtp_raw_test_init(u32 check_types)
{
    u16 i = 0;
    
    test_rslt_buf = (s32*) kmalloc(sizeof(s32)*sample_set_num, GFP_ATOMIC);	
    touchpad_sum = (struct gt9xx_open_info*) kmalloc(sizeof(struct gt9xx_open_info) * 4 * _BEYOND_REC_MAX, GFP_ATOMIC);
    if (NULL == test_rslt_buf || touchpad_sum == NULL)
    {
        SET_INFO_LINE_ERR("Test result buffer allocate failed!");
    }
    memset(touchpad_sum, 0, sizeof(struct gt9xx_open_info) * 4 * _BEYOND_REC_MAX);
    for (i = 0; i < gt9xx_drv_num*gt9xx_sen_num; i++)
    {
        if (i < sample_set_num)
        {
            test_rslt_buf[i] = _CHANNEL_PASS;
        }
    }
    
#if AREA_ACCORD_CHECK
    AreaCheckResult=0;
    for (i = 0; i < MAX_SENSOR_NUM * MAX_DRIVER_NUM; i++)
    {
        channel_status[i]=0;
        beyond_accord_limit_num[i]=0;
    }
#endif
#if ALL_ACCORD_CHECK
    AllCheckResult=0;
    for (i = 0; i < MAX_SENSOR_NUM * MAX_DRIVER_NUM; i++)
    {
        all_channel_status[i]=0;
        beyond_all_accord_limit_num[i]=0;
    }
#endif
}

/*
*********************************************************************************************************
* Function: 
*	touchscreen rawdata min limit test
* Input:
*	u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static void gtp_raw_min_test(u16 *raw_buf)
{
    u16 i, j=0;
    u8 driver, sensor;
    u8 sum_base = 1 * _BEYOND_REC_MAX;
    u8 new_flag = 0;
    
    for (i = 0; i < gt9xx_sc_pxl_cnt; i++)
    {
        if (raw_buf[i] < min_limit_value)
        {
            test_rslt_buf[rslt_buf_idx] |= _BEYOND_MIN_LIMIT;		
            driver = (i/gt9xx_sen_num) + 1;
            sensor = (i%gt9xx_sen_num) + 1;
            new_flag = 0;
            for (j = sum_base; j < (sum_base+_BEYOND_REC_MAX); ++j)
            {
                if (touchpad_sum[j].driver == 0)
                {
                    new_flag = 1;
                    break;
                }
                if ((driver == touchpad_sum[j].driver) && (sensor == touchpad_sum[j].sensor))
                {
                    touchpad_sum[j].times++;
                    new_flag = 0;
                    break;
                }
            }
            if (new_flag)	// new one
            {
                touchpad_sum[j].driver = driver;
                touchpad_sum[j].sensor = sensor;
                touchpad_sum[j].beyond_type |= _BEYOND_MIN_LIMIT;
                touchpad_sum[j].raw_val = raw_buf[i];
                touchpad_sum[j].times = 1;
            }
            else
            {
                continue;
            }
            GTP_DEBUG("[%d, %d]rawdata: %d, raw min limit: %d", driver, sensor, raw_buf[i], min_limit_value);
        }
    }
}

/*
*********************************************************************************************************
* Function: 
*	touchscreen rawdata max limit test
* Input:
*	u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static void gtp_raw_max_test(u16 *raw_buf)
{
    u16 i, j;
    u8 driver, sensor;
    u8 sum_base = 0 * _BEYOND_REC_MAX;
    u8 new_flag = 0;
    
    for (i = 0; i < gt9xx_sc_pxl_cnt; i++)
    {
        if (raw_buf[i] > max_limit_value)
        {
            test_rslt_buf[rslt_buf_idx] |= _BEYOND_MAX_LIMIT;    	
            driver = (i/gt9xx_sen_num) + 1;
            sensor = (i%gt9xx_sen_num) + 1;
            new_flag = 0;
            for (j = sum_base; j < (sum_base+_BEYOND_REC_MAX); ++j)
            {
                if (touchpad_sum[j].driver == 0)
                {
                    new_flag = 1;
                    break;
                }
                if ((driver == touchpad_sum[j].driver) && (sensor == touchpad_sum[j].sensor))
                {
                    touchpad_sum[j].times++;
                    new_flag = 0;
                    break;
                }
            }
            if (new_flag)	// new one
            {
                touchpad_sum[j].driver = driver;
                touchpad_sum[j].sensor = sensor;
                touchpad_sum[j].beyond_type |= _BEYOND_MAX_LIMIT;
                touchpad_sum[j].raw_val = raw_buf[i];
                touchpad_sum[j].times = 1;
            }
            else
            {
                continue;
            }
            GTP_DEBUG("[%d, %d]rawdata: %d, raw max limit: %d", driver, sensor, raw_buf[i], max_limit_value);
        }
    }
}

#if GTP_HAVE_TOUCH_KEY
/*
*********************************************************************************************************
* Function: 
*	key rawdata max limit test
* Input:
*	u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static void gtp_key_max_test(u16 *raw_buf)
{
    u16 i = 0, j = 1, k = 0;
    u8 key_cnt = key_iso_pos[0];
    u8 driver, sensor;
    u8 sum_base = 2 * _BEYOND_REC_MAX;
    u8 new_flag = 0;
    
    driver = gt9xx_drv_num;
    for (i = gt9xx_sc_pxl_cnt; i < gt9xx_pixel_cnt; ++i)
    {
        sensor = (i%gt9xx_sen_num) + 1;
        if (key_is_isolated)
        { 
            if ((key_iso_pos[j] != sensor) || (key_cnt == 0))
            {
                continue;
            }
            else	// only test key pixel rawdata
            {
                --key_cnt;
                ++j;
            }
        }
        if (raw_buf[i] > max_limit_key)
        {
            test_rslt_buf[rslt_buf_idx] |= _BEYOND_KEY_MAX_LMT;
            new_flag = 0;
            for (k = sum_base; k < (sum_base+_BEYOND_REC_MAX); ++k)
            {
                if (touchpad_sum[k].driver == 0)
                {
                    new_flag = 1;
                    break;
                }
                if (touchpad_sum[k].sensor == sensor)
                {
                    touchpad_sum[k].times++;
                    new_flag = 0;
                    break;
                }
            }
            if (new_flag)	// new one
            {
                touchpad_sum[k].driver = driver;
                touchpad_sum[k].sensor = sensor;
                touchpad_sum[k].beyond_type |= _BEYOND_KEY_MAX_LMT;
                touchpad_sum[k].raw_val = raw_buf[i];
                touchpad_sum[k].times = 1;
                if (key_is_isolated)
                {
                    touchpad_sum[k].key = j-1;
                }
            }
            else
            {
                continue;
            }
            GTP_DEBUG("[%d, %d]key rawdata: %d, key max limit: %d", driver,sensor, raw_buf[i], max_limit_key);
        }
    }
}
/*
*********************************************************************************************************
* Function: 
*	key rawdata min limit test
* Input:
*	u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
void gtp_key_min_test(u16 *raw_buf)
{
    u16 i = 0, j = 1, k = 0;
    u8 key_cnt = key_iso_pos[0];
    u8 driver, sensor;
    u8 sum_base = 3 * _BEYOND_REC_MAX;
    u8 new_flag = 0;
    
    driver = gt9xx_drv_num;
    for (i = gt9xx_sc_pxl_cnt; i < gt9xx_pixel_cnt; ++i)
    {
        sensor = (i%gt9xx_sen_num) + 1;
        if (key_is_isolated)
        {
            if ((key_iso_pos[j] != sensor) || (key_cnt == 0))
            {
                continue;
            }
            else	// only test key pixel rawdata
            {
                --key_cnt;
                ++j;
            }
        }
    
        if (raw_buf[i] < min_limit_key)
        {
            test_rslt_buf[rslt_buf_idx] |= _BEYOND_KEY_MIN_LMT;
            new_flag = 0;
            for (k = sum_base; k < (sum_base + _BEYOND_REC_MAX); ++k)
            {
                if (touchpad_sum[k].driver == 0)
                {
                    new_flag = 1;
                    break;
                }
                if (sensor == touchpad_sum[k].sensor)
                {
                    touchpad_sum[k].times++;
                    break;
                }
            }
            if (new_flag)	// new one
            {
                touchpad_sum[k].driver = driver;
                touchpad_sum[k].sensor = sensor;
                touchpad_sum[k].beyond_type |= _BEYOND_KEY_MIN_LMT;
                touchpad_sum[k].raw_val = raw_buf[i];
                touchpad_sum[k].times = 1;
                if (key_is_isolated)
                {
                    touchpad_sum[k].key = j-1;
                }
            }
            else
            {
                continue;
            }
            GTP_DEBUG("[%d, %d]key rawdata: %d, key min limit: %d", driver, sensor, raw_buf[i], min_limit_key);
        }
    }
}
#endif
/*
*********************************************************************************************************
* Function: 
*	analyse rawdata retrived from ic registers
* Input:
*	u16 *raw_buf, buffer for rawdata, 
*   u32 check_types, test items
* Return:
*	SUCCESS: test process succeed, FAIL: failed
*********************************************************************************************************
*/
static u32 gtp_raw_test(u16 *raw_buf, u32 check_types)
{	
    if (raw_buf == NULL)
    {
        GTP_DEBUG("Invalid raw buffer pointer!");
        return FAIL;
    } 
    if (0 == check_types)
    {
        check_types = default_test_types;
    #if GTP_HAVE_TOUCH_KEY
        check_types |= _KEY_MAX_TEST | _KEY_MIN_TEST;
    #endif
    #if AREA_ACCORD_CHECK
        check_types |= _AREA_TEST;
    #endif
    #if ALL_ACCORD_CHECK
        check_types |= _ALL_TEST;
    #endif
    }
    
    if (check_types & _MAX_TEST)
    {
        gtp_raw_max_test(raw_buf);
    }

    if (check_types & _MIN_TEST)	
    {
        gtp_raw_min_test(raw_buf);	
    }
#if GTP_HAVE_TOUCH_KEY
    if (check_types & _KEY_MAX_TEST)	
    {
        gtp_key_max_test(raw_buf);
    }
    if (check_types & _KEY_MIN_TEST)
    {
        gtp_key_min_test(raw_buf);
    }
#endif

#if AREA_ACCORD_CHECK
    if (check_types & _AREA_TEST)
    {
        AreaAccordCheck(raw_buf);
    }
#endif
#if ALL_ACCORD_CHECK
    if (check_types & _ALL_TEST)
    {
        AllAccordCheck(raw_buf);
    }
#endif
    return SUCCESS;
} 


/*
====================================================================================================
* Function: 
* 	output the test result
* Return: 
* 	return the result. if result == 0, the TP is ok, otherwise list the beyonds
====================================================================================================
*/

static s32 gtp_get_test_result(void)
{
    u16 i = 0, j = 0;
    u16 beyond_max_num = 0;			// beyond max limit test times
    u16 beyond_min_num = 0;			// beyond min limit test times
#if GTP_HAVE_TOUCH_KEY
    u16 beyond_key_max = 0;			// beyond key max limit test times
    u16 beyond_key_min = 0;			// beyond key min limit test times
#endif
    s32 result = _CHANNEL_PASS;
    
#if GTP_DEBUG_ON

    for (i = 0; i < 4 * _BEYOND_REC_MAX; ++i)
    {
        printk("(%2d, %2d)[%2d] ", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
        if (i && ((i+1) % 5 == 0))
        {
            printk("\n");
        }
    }
    printk("\n");

#endif

    for (i = 0; i < sample_set_num; ++i)
    {
        if (test_rslt_buf[i] & _BEYOND_MAX_LIMIT) 
        {
            beyond_max_num++;
        }
        if (test_rslt_buf[i] & _BEYOND_MIN_LIMIT)
        {
            beyond_min_num++;
        }
    #if GTP_HAVE_TOUCH_KEY
        if (test_rslt_buf[i] & _BEYOND_KEY_MAX_LMT)
        {
            beyond_key_max++;
        }
        if (test_rslt_buf[i] & _BEYOND_KEY_MIN_LMT)
        {
            beyond_key_min++;
        }
#endif
    }
    if (beyond_max_num > _MIN_ERROR_NUM)
    {
        result |= _BEYOND_MAX_LIMIT;
        j = 0;
        SET_INFO_LINE_INFO("Beyond Max Limit Points Info: ");
        for (i = 0; i < _BEYOND_REC_MAX; ++i)
        {
            if (touchpad_sum[i].driver == 0)
            {
                break;
            }
           // SET_INFO_LINE_INFO("Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
	    GTP_INFO("Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
	    msleep(10);
        }
    }
    if (beyond_min_num > _MIN_ERROR_NUM)
    {
        result |= _BEYOND_MIN_LIMIT;
        SET_INFO_LINE_INFO("Beyond Min Limit Points Info:");
        j = 0;
        for (i = _BEYOND_REC_MAX; i < (2*_BEYOND_REC_MAX); ++i)
        {
            if (touchpad_sum[i].driver == 0)
            {
                break;
            }
            //SET_INFO_LINE_INFO("  Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
	      GTP_INFO("  Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
	      msleep(10);
        }
    }
#if GTP_HAVE_TOUCH_KEY
    if (beyond_key_max > _MIN_ERROR_NUM)
    {
        result |= _BEYOND_KEY_MAX_LMT;
        SET_INFO_LINE_INFO("Beyond Key Max Limit Key Info:");
        for (i = 2*_BEYOND_REC_MAX; i < (3*_BEYOND_REC_MAX); ++i)
        {
            if (touchpad_sum[i].driver == 0)
            {
                break;
            }
            //SET_INFO_LINE_INFO("  Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
	      GTP_INFO("  Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
		 msleep(10);
        }
    }
    if (beyond_key_min > _MIN_ERROR_NUM)
    {
        result |= _BEYOND_KEY_MIN_LMT;		 
        SET_INFO_LINE_INFO("Beyond Key Min Limit Key Info:");
        for (i = 3*_BEYOND_REC_MAX; i < (4*_BEYOND_REC_MAX); ++i)
        {
            if (touchpad_sum[i].driver == 0)
            {
                break;
            }
            SET_INFO_LINE_INFO("  Drv: %d, Sen: %d[Times: %d]", touchpad_sum[i].driver, touchpad_sum[i].sensor, touchpad_sum[i].times);
        }
    }
#endif

#if AREA_ACCORD_CHECK
    if (AreaCheckResult)
    {
        i = 0;
        SET_INFO_LINE_INFO("Beyond Area Accord Check Info:");
        for (j = 0; j < MAX_SENSOR_NUM * MAX_DRIVER_NUM; j++)			
        {
            if (channel_status[j] & _BEYOND_ACCORD_LIMIT)
            {
                if (i < _BEYOND_REC_MAX)
                {
                    i++;
                   // SET_INFO_LINE_INFO("  [%d]Ch: [%d], T: [%d], Val: [%d]", i, j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
		    GTP_INFO("  [%d]Ch: [%d], T: [%d], Val: [%d]", i, j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
		     msleep(10);
                }
            }
        }
        if (i == _BEYOND_REC_MAX)
        {
            SET_INFO_LINE_INFO("  More...........................................");
        }

    }
    result |= AreaCheckResult;
#endif

#if ALL_ACCORD_CHECK
    if (AllCheckResult)
    {
        i = 0;
        SET_INFO_LINE_INFO("Beyond All Accord Check Info:");
        for (j = 0; j < MAX_SENSOR_NUM * MAX_DRIVER_NUM; j++)			
        {
            if (all_channel_status[j] & _BEYOND_ALL_ACCORD_LIMIT)
            {
                if (i < _BEYOND_REC_MAX)
                {
                    i++;
                    //SET_INFO_LINE_INFO("  [%d]Ch: [%d], T: [%d], Val: [%d]", i, j, beyond_all_accord_limit_num[j], beyond_all_accord_limit_val[j]);
		      GTP_INFO("  [%d]Ch: [%d], T: [%d], Val: [%d]", i, j, beyond_all_accord_limit_num[j], beyond_all_accord_limit_val[j]);
		       msleep(10);
                }
            }
        }
        if (i == _BEYOND_REC_MAX)
        {
            SET_INFO_LINE_INFO("  More...........................................");
        }

    }
    result |= AllCheckResult;
#endif
    
    if (result == 0)
    {	opentestflag = 1;
        SET_INFO_LINE_INFO("[TEST SUCCEED]: The TP is ok!");
        return result;
    }
//#if AREA_ACCORD_CHECK
//#if ALL_ACCORD_CHECK
//    if ((result == _BEYOND_ACCORD_LIMIT) ||  (result == _BEYOND_ALL_ACCORD_LIMIT))
//    {
//        SET_INFO_LINE_INFO("[TEST SUCCEED]: The TP is conditional ok!");
//        SET_INFO_LINE_INFO("PASS!PASS!PASS!PASS!PASS!");	//<ASUS+>
//        return result;
//    }
//#endif
//#endif
	
    SET_INFO_LINE_INFO("[TEST FAILED]:");
    if (result & _BEYOND_MAX_LIMIT)
    {
        SET_INFO_LINE_INFO("  Beyond Raw Max Limit[Max Limit: %d]", max_limit_value);
    }
    if (result & _BEYOND_MIN_LIMIT)
    {
        SET_INFO_LINE_INFO("  Beyond Raw Min Limit[Min Limit: %d]", min_limit_value);
    }
#if GTP_HAVE_TOUCH_KEY
    if (result & _BEYOND_KEY_MAX_LMT)
    {
        SET_INFO_LINE_INFO("  Beyond KeyVal Max Limit[Key Max Limit: %d]", max_limit_key);
    }
    if (result & _BEYOND_KEY_MIN_LMT)
    {
        SET_INFO_LINE_INFO("  Beyond KeyVal Min Limit[Key Min Limit: %d]", min_limit_key);
    }
#endif
#if AREA_ACCORD_CHECK
    if (result & _BEYOND_ACCORD_LIMIT)
    {
        SET_INFO_LINE_INFO("  Area Accord Check failed[Accord Limit: %d]", accord_limit);
    }
#endif
#if ALL_ACCORD_CHECK
    if (result & _BEYOND_ALL_ACCORD_LIMIT)
    {
        SET_INFO_LINE_INFO("  All Accord Check failed[Average: %d, All Accord Limit: %d]", average, all_accord_limit);
    }
#endif
    return result;
}

/*
 ===================================================
 * Function: 
 * 		test gt9 series ic open test
 * Input:
 * 		client, i2c_client
 * Return:
 * 		SUCCESS: test process success, FAIL, test process failed
 *
 ===================================================
*/
	
s32 gt9xx_open_test(struct i2c_client * client)
{
    u16 i = 0;
    s32 ret = 0; // SUCCESS, FAIL
    struct goodix_ts_data *ts = NULL;
    u16 *raw_buf = NULL;
    u8 rd_cfg_buffer[3];
    int retry = 0;
    
    ts = i2c_get_clientdata(i2c_connect_client);
    gtp_irq_disable(ts);
    SET_INFO_LINE_INFO("---gtp open test---");

    rd_cfg_buffer[0] = GTP_REG_SENSOR_ID >> 8;
    rd_cfg_buffer[1] = GTP_REG_SENSOR_ID & 0xff;
//ASUStoby-20130731
    while(retry < 3){
    	ret = gtp_i2c_read(client, rd_cfg_buffer, 3);
		if(ret >= 0)
			break;

		retry++;
    }
//ASUStoby-20130731
    if (ret < 0)
    {
        SET_INFO_LINE_ERR("Read SENSOR ID failed,use default limit!");
#if AREA_ACCORD_CHECK
        SET_INFO_LINE_ERR("Read SENSOR ID failed,use default accord_limit!");
#endif
#if ALL_ACCORD_CHECK
        SET_INFO_LINE_ERR("Read SENSOR ID failed,use default all_accord_limit!");
#endif
    }
    rd_cfg_buffer[GTP_ADDR_LENGTH] &= 0x07;
    if (rd_cfg_buffer[GTP_ADDR_LENGTH] == 5)        //Jtouch
    {
        max_limit_value = max_limit_value_Jtouch;
        min_limit_value = min_limit_value_Jtouch;
#if AREA_ACCORD_CHECK
        accord_limit = accord_limit_Jtouch;
#endif
#if ALL_ACCORD_CHECK
        all_accord_limit = all_accord_limit_Jtouch;
#endif
    }
    else if (rd_cfg_buffer[GTP_ADDR_LENGTH] == 4)   //Ofilm
    {
        max_limit_value = max_limit_value_Ofilm;
        min_limit_value = min_limit_value_Ofilm;
#if AREA_ACCORD_CHECK
        accord_limit = accord_limit_Ofilm;
#endif
#if ALL_ACCORD_CHECK
        all_accord_limit = all_accord_limit_Ofilm;
#endif
    }

    SET_INFO_LINE_INFO("Max Limit Value: %d", max_limit_value);
    SET_INFO_LINE_INFO("Min Limit Value: %d", min_limit_value);
#if AREA_ACCORD_CHECK
    SET_INFO_LINE_INFO("Area Accord Limit: %d", accord_limit);
#endif
#if ALL_ACCORD_CHECK
    SET_INFO_LINE_INFO("All Accord Limit: %d", all_accord_limit);
#endif

    SET_INFO_LINE_INFO("---GT9xx Open Test (ID:%d)---", rd_cfg_buffer[GTP_ADDR_LENGTH]);

    GTP_DEBUG("Parsing configuration...");
    ret = gtp_parse_config();
    if (ret == FAIL)
    {
        SET_INFO_LINE_ERR("failed to parse config...");
        goto open_test_exit;
    }
    raw_buf = (u16*)kmalloc(sizeof(u16)* gt9xx_pixel_cnt, GFP_KERNEL);
    if (NULL == raw_buf)
    {
        SET_INFO_LINE_ERR("failed to allocate mem for raw_buf!");
        goto open_test_exit;
    }

    GTP_DEBUG("Step 1: Send Rawdata Cmd");
    
    gtp_rawdiff_mode = 1;
    ts->gtp_rawdiff_mode = 1;
    gtp_raw_test_init(0);
    ret = gt9_read_raw_cmd(client);
    if (ret == FAIL)
    {
        SET_INFO_LINE_ERR("Send Read Rawdata Cmd failed!");
        goto open_test_exit;
    }

    GTP_DEBUG("Step 2: Sample Rawdata");
    for (i = 0; i < sample_set_num; ++i)
    {	
        rslt_buf_idx = i;
        ret = gtp_read_rawdata(client, raw_buf);
        if (ret == FAIL)
        {
            SET_INFO_LINE_ERR("Read Rawdata failed!");
            goto open_test_exit;
        }
        ret = gtp_raw_test(raw_buf, 0);
        if (ret == FAIL)
        {
            gtp_i2c_end_cmd(client);
            continue;
        }
    }

    GTP_DEBUG("Step 3: Analyse Result");
    SET_INFO_LINE_INFO("Total %d Sample Data", sample_set_num);
    gtp_get_test_result();
  

    ret = SUCCESS;
open_test_exit:
    
    kfree(raw_buf);
    if (test_rslt_buf)
    {
        kfree(test_rslt_buf);
    }
    if (touchpad_sum)
    {
        kfree(touchpad_sum);
    }
    gtp_irq_enable(ts);
    ts->gtp_rawdiff_mode = 0;
    gt9_read_coor_cmd(client);	// back to read coordinates data 
    SET_INFO_LINE_INFO("---gtp open test end---");
    gup_i2c_write(ts->client, config, GTP_ADDR_LENGTH + GTP_CONFIG_MAX_LENGTH);
    return ret;
}

static ssize_t gtp_sysfs_shorttest_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    u32 index;
    u32 len;
    
    gt9xx_short_test(i2c_connect_client) ;
		
    
    
    for (index = 0, len = 0; index < RsltIndex; ++index)
    {
        sprintf(&buf[len], "%s", result_lines[index]);
        len += strlen(result_lines[index]);
        kfree(result_lines[index]);
    }
    RsltIndex = 0;
    return len;
}

static ssize_t gtp_sysfs_shorttest_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t gtp_sysfs_opentest_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    s32 index;
    u32 len = 0;
    
    gt9xx_open_test(i2c_connect_client);
		
    
    for (index = 0, len = 0; index < RsltIndex; ++index)
    {
        sprintf(&buf[len], "%s", result_lines[index]);
        len += strlen(result_lines[index]);
        kfree(result_lines[index]);
    }
    RsltIndex = 0;
    return len;
}

static ssize_t gtp_sysfs_opentest_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    char *tbuf;
    int error;
    unsigned int val;

    // Jtouch
    // adb shell "echo 5 1800 1200 20000 20000> /sys/gtp_test/opentest"\
	// Ofilm ???

    printk("Set ID Max Min Area All Limit Value for Opentest\n");

    tbuf = buf;
    *(tbuf+1) = '\0';
    *(tbuf+6) = '\0';
#if AREA_ACCORD_CHECK
    *(tbuf+11) = '\0';
#endif
#if ALL_ACCORD_CHECK
    *(tbuf+17) = '\0';
#endif

    error = kstrtouint(tbuf, 10, &val);
    if (error)
    {
        printk("Set ID Fail!\n");
        return error;
    }
    printk("Set ID: %d\n", val);

    if (val == 5)    //Jtouch
    {
        error = kstrtouint((tbuf+2), 10, &val);
        if (error)
        {
            printk("Set Max Limit Value Fail!\n");
            return error;
        }
        max_limit_value_Jtouch = val;

        error = kstrtouint((tbuf+7), 10, &val);
        if (error)
        {
            printk("Set Min Limit Value Fail!\n");
            return error;
        }
        min_limit_value_Jtouch = val;

#if AREA_ACCORD_CHECK
        error = kstrtouint((tbuf+12), 10, &val);
        if (error)
        {
            printk("Set Area Accord Limit Fail!\n");
            return error;
        }
        accord_limit_Jtouch = val;
#endif
#if ALL_ACCORD_CHECK
        error = kstrtouint((tbuf+18), 10, &val);
        if (error)
        {
            printk("Set All Accord Limit Fail!\n");
            return error;
        }
        all_accord_limit_Jtouch = val;
#endif
    }
    else if (val == 4)    //Ofilm
    {
     error = kstrtouint((tbuf+2), 10, &val);
        if (error)
        {
            printk("Set Max Limit Value Fail!\n");
            return error;
        }
        max_limit_value_Ofilm = val;

        error = kstrtouint((tbuf+7), 10, &val);
        if (error)
        {
            printk("Set Min Limit Value Fail!\n");
            return error;
        }
        min_limit_value_Ofilm = val;

#if AREA_ACCORD_CHECK
        error = kstrtouint((tbuf+12), 10, &val);
        if (error)
        {
            printk("Set Area Accord Limit Fail!\n");
            return error;
        }
        accord_limit_Ofilm = val;
#endif
#if ALL_ACCORD_CHECK
        error = kstrtouint((tbuf+18), 10, &val);
        if (error)
        {
            printk("Set All Accord Limit Fail!\n");
            return error;
        }
        all_accord_limit_Ofilm = val;
#endif
    
    }

    printk("Set Max Min Limit Value Success!\n");
    printk("Max Limit Value: %d\n", max_limit_value);
    printk("Min Limit Value: %d\n", min_limit_value);
#if AREA_ACCORD_CHECK
    printk("Area Accord Limit: %d\n", accord_limit);
#endif
#if ALL_ACCORD_CHECK
    printk("All Accord Limit: %d\n", all_accord_limit);
#endif

    return count;
    //return -EPERM;
}

static ssize_t gtp_sysfs_max_limit_value_show(struct device *dev,struct device_attribute *attr, char *buf)
{   
    return sprintf(buf,"%d\n", max_limit_value);
}

static ssize_t gtp_sysfs_max_limit_value_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    max_limit_value = simple_strtoul(buf,NULL,10);
    return strlen(buf);
}

static ssize_t gtp_sysfs_min_limit_value_show(struct device *dev,struct device_attribute *attr, char *buf)
{   
    return sprintf(buf,"%d\n", min_limit_value);
}

static ssize_t gtp_sysfs_min_limit_value_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    min_limit_value = simple_strtoul(buf,NULL,10);
    return strlen(buf);
}

//ASUS -> toby 20130711
static ssize_t gtp_sysfs_openteststat_show(struct device *dev,struct device_attribute *attr, char *buf)
{	
	u32 index;
    u32 len;
    
   if(opentestflag == 1){
		SET_INFO_LINE_INFO("Open test PASS!");
	}else
	{
		SET_INFO_LINE_INFO("Open test FAIL!");
	}
    
    
    for (index = 0, len = 0; index < RsltIndex; ++index)
    {
        sprintf(&buf[len], "%s", result_lines[index]);
        len += strlen(result_lines[index]);
        kfree(result_lines[index]);
    }
    RsltIndex = 0;
    return len; 
  
}

static ssize_t gtp_sysfs_openteststat_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    opentestflag = simple_strtoul(buf,NULL,10);
    return strlen(buf);
}

static ssize_t gtp_sysfs_shortteststat_show(struct device *dev,struct device_attribute *attr, char *buf)
{   
	u32 index;
    u32 len;
    if(shorttestflag == 1){
		SET_INFO_LINE_INFO("Short test PASS!");
	}else
	{
		SET_INFO_LINE_INFO("Short test FAIL!");
	}

	for (index = 0, len = 0; index < RsltIndex; ++index)
    {
        sprintf(&buf[len], "%s", result_lines[index]);
        len += strlen(result_lines[index]);
        kfree(result_lines[index]);
    }
    RsltIndex = 0;
    return len;
}

static ssize_t gtp_sysfs_shortteststat_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    shorttestflag = simple_strtoul(buf,NULL,10);
    return strlen(buf);
}
//ASUS -> toby 20130711

//ASUS -> toby 20130711
static DEVICE_ATTR(openteststat, S_IRUGO|S_IWUSR, gtp_sysfs_openteststat_show, gtp_sysfs_openteststat_store);
static DEVICE_ATTR(shortteststat, S_IRUGO|S_IWUSR, gtp_sysfs_shortteststat_show, gtp_sysfs_shortteststat_store);
//ASUS -> toby 20130711

static DEVICE_ATTR(shorttest, S_IRUGO|S_IWUSR, gtp_sysfs_shorttest_show, gtp_sysfs_shorttest_store);
static DEVICE_ATTR(opentest, S_IRUGO|S_IWUSR, gtp_sysfs_opentest_show, gtp_sysfs_opentest_store);
static DEVICE_ATTR(max_limit_value, S_IRUGO|S_IWUSR, gtp_sysfs_max_limit_value_show, gtp_sysfs_max_limit_value_store);
static DEVICE_ATTR(min_limit_value, S_IRUGO|S_IWUSR, gtp_sysfs_min_limit_value_show, gtp_sysfs_min_limit_value_store);

/*******************************************************
Description:
	Goodix debug sysfs init function.

Parameter:
	none.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
s32 gtp_test_sysfs_init(void)
{
    s32 ret ;

    goodix_debug_kobj = kobject_create_and_add("gtp_test", NULL) ;
    SET_INFO_LINE_INFO("Starting initlizing gtp_debug_sysfs");
    if (goodix_debug_kobj == NULL)
    {
        GTP_ERROR("%s: subsystem_register failed\n", __func__);
        return -ENOMEM;
    }

    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_shorttest.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_opentest.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_max_limit_value.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs create max_limit_value failed\n", __func__);
        return ret;
    }
    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_min_limit_value.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs create min_limit_value failed\n", __func__);
        return ret;
    }
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_openteststat.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs create openteststat failed\n", __func__);
        return ret;
    }
    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_shortteststat.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs create shortteststat failed\n", __func__);
        return ret;
    }

    GTP_INFO("Goodix debug sysfs create success!\n");
    return 0 ;
}

void gtp_test_sysfs_deinit(void)
{
    sysfs_remove_file(goodix_debug_kobj, &dev_attr_shorttest.attr);
    sysfs_remove_file(goodix_debug_kobj, &dev_attr_opentest.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_shortteststat.attr);
    sysfs_remove_file(goodix_debug_kobj, &dev_attr_openteststat.attr);
    kobject_del(goodix_debug_kobj);
}






