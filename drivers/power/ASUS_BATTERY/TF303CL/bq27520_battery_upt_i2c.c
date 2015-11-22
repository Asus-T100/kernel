/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/idr.h>

#include "asus_battery.h"
#include "bq27520_battery_core.h"
#include "bq27520_proc_fs.h"
#include "smb347_external_include.h"

#define RETRY_COUNT 3


extern struct battery_dev_info bq27520_dev_info;
extern struct mutex bq27520_dev_info_mutex;
extern struct bq27xx_dffs_data g4_bq27xx_fw[];
extern const unsigned int g4_num_of_op_buf;
const unsigned int g3_num_of_op_buf = NULL;
extern struct battery_info_reply batt_info;
extern int  flash_gauge_status;

struct battery_dev_info batt_upt_dev_info;
DEFINE_MUTEX(batt_dev_upt_mutex);

int bq27520_rom_mode_write_i2c(u8 reg, int value, int b_single)
{
    struct i2c_client *i2c = NULL;

    mutex_lock(&batt_dev_upt_mutex);
    i2c = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    if (!i2c|| !i2c->adapter)
        return -ENODEV;

    return bq27520_write_i2c(i2c, reg, value, b_single);
}

int bq27520_rom_mode_read_i2c(u8 reg, int *rt_value, int b_single)
{
    struct i2c_client *i2c = NULL;

    mutex_lock(&batt_dev_upt_mutex);
    i2c = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    if (!i2c|| !i2c->adapter)
        return -ENODEV;

    return bq27520_read_i2c(i2c, reg, rt_value, b_single);
}

int bq27520_is_rom_mode(void)
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    BAT_DBG_E("[%s] enter \n", __func__);

    mutex_lock(&batt_dev_upt_mutex);
    i2c = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    while (retry--) {
        ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (ret < 0) {
        return 0; //not rom mode
    }
    return 1; //it's rom mode
}

int bq27520_enter_rom_mode()
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    BAT_DBG_E("[%s] enter \n", __func__);

    if (bq27520_is_rom_mode()) {
        return 0;
    }

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    while (retry--) {
        ret = bq27520_write_i2c(i2c, 0x00, 0x0F00, 0);
        if (ret < 0) continue;

        break;
    };
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }

    /*
     * verify it's ROM node.
     * Yes if read registers FAIL from now on.
     */
    ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
    if (!ret)  {
        BAT_DBG_E("Exit ROM Mode verification FAIL.\n");
        return -EACCES;
    }

    return 0;
}

int bq27520_write_i2c_retry (struct i2c_client *i2c, int addr, int value) {
   int retry = RETRY_COUNT;
   int ret = 0;
   while (retry--) {
      ret = bq27520_write_i2c(i2c, addr, value, 1);
      if (ret < 0) continue;

      break;
   };

   return ret;
}

int bq27520_default_unseal() {
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    BAT_DBG_E("[%s] enter \n", __func__);

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    ret = bq27520_write_i2c_retry(i2c, 0x00, 0x14);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0x04);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0x72);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0x36);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    bq27520_rom_mode_wait(20);
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    bq27520_rom_mode_wait(20);
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0x00);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0x0f);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    bq27520_rom_mode_wait(20);

    /*
     * verify it's ROM mode.
     * Yes if read registers FAIL from now on.
     */
    ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
    if (!ret)  {
        BAT_DBG_E("Exit ROM Mode verification FAIL.\n");
        return -EACCES;
    }

    return 0;
}

int bq27520_ME560CG_unseal() {
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    BAT_DBG_E("[%s] enter \n", __func__);

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    ret = bq27520_write_i2c_retry(i2c, 0x00, 0x14);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0x04);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0x72);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0x16);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    bq27520_rom_mode_wait(20);
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0xff);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    bq27520_rom_mode_wait(20);
    ret = bq27520_write_i2c_retry(i2c, 0x00, 0x00);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    ret = bq27520_write_i2c_retry(i2c, 0x01, 0x0f);
    if (ret < 0) {
        BAT_DBG_E("Enter ROM mode FAIL \n");
        return ret;
    }
    bq27520_rom_mode_wait(20);

    /*
     * verify it's ROM mode.
     * Yes if read registers FAIL from now on.
     */
    ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
    if (!ret)  {
        BAT_DBG_E("Exit ROM Mode verification FAIL.\n");
        return -EACCES;
    }

    return 0;
}

int bq27520_exit_rom_mode()
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    struct i2c_client *client = NULL;
    int val=0;
    int ret=0;

    mutex_lock(&batt_dev_upt_mutex);
    i2c = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    BAT_DBG_E("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_write_i2c(i2c, 0x00, 0x0F, 1);
        if (ret < 0) continue;

        ret = bq27520_write_i2c(i2c, 0x64, 0x0F, 1);
        if (ret < 0) continue;

        ret = bq27520_write_i2c(i2c, 0x65, 0x00, 1);
        if (ret < 0) continue;

        break;
    };
    if (!retry) {
        BAT_DBG_E("Exit ROM mode FAIL \n");
        return ret;
    }

    /*
     * verify it's NOT ROM node.
     * Yes if read registers FAIL from now on.
     */
    ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
    if (!ret)  {
        BAT_DBG_E("Exit ROM Mode verification FAIL.\n");
        return -EACCES;
    }

    /*
     * wait 1s and send IT_ENABLE command
     * (battery team request)
     */
    msleep(1000);

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    ret = bq27520_send_subcmd(client, NULL, BQ27520_SUBCMD_ENABLE_IT);
    if (ret)
        return ret;

    return 0;
}

int bq27520_rom_mode_cmp(int reg_off, int value)
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    mutex_lock(&batt_dev_upt_mutex);
    i2c = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    BAT_DBG_E("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(i2c, reg_off, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (!retry && ret < 0) {
        return ret;
    }

    return val == value ? PROC_TRUE : PROC_FALSE;
}

int bq27520_rom_mode_wait(int m_secs)
{
    BAT_DBG_E("[%s] enter \n", __func__);

    if (m_secs < 1) return -EINVAL;

    msleep(m_secs);

    return 0;
}

int update_fw(struct update_op *p_start, struct update_op *p_end)
{
    int ret=UPDATE_OK;
    struct i2c_client *i2c_upt = NULL;
    struct i2c_client *i2c_dev = NULL;

    BAT_DBG("(%s) Start update firmware ... ", __func__);

    mutex_lock(&bq27520_dev_info_mutex);
    i2c_dev = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    mutex_lock(&batt_dev_upt_mutex);
    i2c_upt = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    ret = UPDATE_PROCESS_FAIL;
    while (p_start <= p_end)
    {
        int op_execute_ret = 0;

#if 0
        /* print update command */
        BAT_DBG("OP: %d, off: %02X, val: %04X\n",
                p_start->bq_op, p_start->off, p_start->arg
        );
#endif

        switch(p_start->bq_op) {
        case OP_ROM_END:
        case OP_ROM_WRITE:
            op_execute_ret = bq27520_rom_mode_write_i2c(p_start->off, p_start->arg, 1);
            if (op_execute_ret < 0) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            }
            break;

        case OP_ROM_CMP:
            op_execute_ret = bq27520_rom_mode_cmp(p_start->off, p_start->arg);
            if (op_execute_ret == PROC_FALSE) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            } else if (op_execute_ret != PROC_FALSE && op_execute_ret != PROC_TRUE) {
                if (op_execute_ret < 0) {
                    BAT_DBG_E("   FAIL\n");
                    return ret;
                }
            }
            break;

        case OP_I2C_START:
        case OP_I2C_WRITE:
            op_execute_ret = bq27520_write_i2c(i2c_dev, p_start->off, p_start->arg, 1);
            if (op_execute_ret < 0) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            }
            break;

        case OP_I2C_CMP:
            op_execute_ret = bq27520_cmp_i2c(p_start->off, p_start->arg);
            if (op_execute_ret != PROC_FALSE && op_execute_ret != PROC_TRUE) {
                BAT_DBG_E("   FAIL %d\n", op_execute_ret);
                return ret;
            } else if (op_execute_ret == PROC_FALSE) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            }
            break;

        case OP_WAIT:
            op_execute_ret = bq27520_rom_mode_wait(p_start->arg);
            if (op_execute_ret < 0) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            }
            break;

        default:
            BAT_DBG("Not support OP \n");
            break;
        }; //end switch

        p_start++;
    }; //end while

    ret = UPDATE_OK;
    BAT_DBG("%s Done.\n", __func__);

    return ret;
}

int update_normal(int curr_cell_type)
{
    int ret;
    struct update_op *p_op_start = NULL, *p_op_end = NULL, *p_op_buf = NULL;
    int i,j;

    BAT_DBG(" fw flashing... please wait for about 10s at least.\n");

    ret = UPDATE_ERR_MATCH_OP_BUF;
    if (batt_info.gauge_version == IC_VERSION_G3) {
        BAT_DBG("bq27520 gauge is G3, no dffs file ignore it.\n");
        ret = UPDATE_NONE;

    } else if (batt_info.gauge_version == IC_VERSION_G4) {
        BAT_DBG("bq27520 gauge is G4\n");
        for (i=0; i<g4_num_of_op_buf; i++) {
            if (curr_cell_type != g4_bq27xx_fw[i].cell_type)
                continue;

            //Skip normal i2c mode op until OP_I2C_START + 1
            p_op_buf = g4_bq27xx_fw[i].op;
            p_op_start = p_op_buf;
            p_op_end = &p_op_buf[g4_bq27xx_fw[i].num_op-1];

            for (j=0; j<RETRY_COUNT; j++) {
                ret = update_fw(p_op_start, p_op_end);
                if (ret == UPDATE_OK) break;
            }
            break;

        }

    }

    return ret;
}

int update_from_normal_mode(bool forceupdate) {

    int curr_cell_type=TYPE_LG, fw_cell_type=TYPE_LG;
    int ret = UPDATE_NONE;
    int curr_volt;
    int fw_cfg_version;

    BAT_DBG("(%s) enter\n", __func__);

#if 0
    curr_cell_type = bq27520_batt_current_sel_type();
#endif
    fw_cell_type = bq27520_batt_fw_sel_type();
    curr_volt = bq27520_asus_battery_dev_read_volt();
    fw_cfg_version = bq27520_asus_battery_dev_read_fw_cfg_version();

    BAT_DBG("(%s) current_cell_type %d, fw_cell_type %d, current_volt %d, fw_cfg_version 0x%04X\n",
            __func__,
            curr_cell_type, fw_cell_type, curr_volt, fw_cfg_version
    );

    if (!forceupdate) {
         if (fw_cfg_version == LATEST_FW_CFG_VERSION) {
                BAT_DBG("========= No need to flash battery cell data due to that both data flash version are equal =========");
                goto Done;
         }
    }

    //Need update, check update voltage
    ret = UPDATE_VOLT_NOT_ENOUGH;
    if (curr_volt < 0 || curr_volt < 3700) {
        BAT_DBG_E("Voltage not enough \n");
        goto Done;
    }

    if (batt_info.gauge_version == IC_VERSION_G3) {
        BAT_DBG("bq27520 gauge is G3, no need enter ROM mode.\n");

    } else if (batt_info.gauge_version == IC_VERSION_G4) {
        //unseal for enter rom mode
        ret = bq27520_default_unseal();
        if (ret < 0) {
           // default password unseal fail
           BAT_DBG_E("bq27520_default_unseal fail !!!!\n");
           bq27520_rom_mode_wait(5000);
           ret = bq27520_ME560CG_unseal();
           if (ret < 0) {
              // ME560CG password unseal fail
              BAT_DBG_E("bq27520_ME560CG_unseal fail !!!!\n");
              ret = UPDATE_CHECK_MODE_FAIL;
              goto Done;
          }
       }
    }

    smb347_charging_toggle(false);
    ret = update_normal(curr_cell_type);
    smb347_charging_toggle(true);

Done:
    flash_gauge_status = ret;
    return ret;
}

int update_from_rom_mode(void)
{
    int cell_type=TYPE_LG;
    int i,j,k;
    int ret;
    struct update_op *p_op_start = NULL, *p_op_end = NULL, *p_op_buf = NULL;

    BAT_DBG(" (%s) enter\n", __func__);

#if 0
    cell_type = bq27520_batt_current_sel_type();
#endif

    BAT_DBG(" fw flashing... please wait for about 30s at least.\n");

    ret = UPDATE_ERR_MATCH_OP_BUF;

    if (batt_info.gauge_version == IC_VERSION_G3) {
        BAT_DBG("bq27520 gauge is G3\n");
        ret = UPDATE_NONE;

    } else if (batt_info.gauge_version == IC_VERSION_G4) {
        BAT_DBG("bq27520 gauge is G4\n");
        for (i=0; i<g4_num_of_op_buf; i++) {
            if (cell_type != g4_bq27xx_fw[i].cell_type)
                continue;

            //Skip normal i2c mode op until OP_I2C_START + 1
            p_op_buf = g4_bq27xx_fw[i].op;
            p_op_end = &p_op_buf[g4_bq27xx_fw[i].num_op-1];
            for (j=0; j<g4_bq27xx_fw[i].num_op; j++) {
#if 0
                BAT_DBG("OP: %d, 0x%02X, 0x%02X\n",
                        p_op_buf[j].bq_op,
                        p_op_buf[j].off,
                        p_op_buf[j].arg
                );
#endif
                if (p_op_buf[j].bq_op >= OP_I2C_START) {
                    k=j;
                    while(p_op_buf[k].bq_op >= OP_I2C_START) k++;
                    p_op_start = &p_op_buf[k];
                    break;
                }
            }

            for (j=0; j<RETRY_COUNT; j++) {
                ret = update_fw(p_op_start, p_op_end);
                if (ret == UPDATE_OK) {
                   ret = UPDATE_FROM_ROM_MODE;
                   break;
                }
            }
            break;

        }

    }



    return ret;
}

int bq27520_bat_upt_main_update_flow(bool forceupdate)
{
    int ret = UPDATE_NONE;

    BAT_DBG_E("[%s] enter \n", __func__);

    if (bq27520_is_rom_mode()) {
        ret = update_from_rom_mode();
    }
    else if (bq27520_is_normal_mode()) {
        ret = update_from_normal_mode(forceupdate);
    }
    else {
        return UPDATE_CHECK_MODE_FAIL;
    }

    return ret;
}

static int __devinit
bq27520_bat_upt_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id) {
    dev_info(&i2c->dev, "%s done.", __func__);

    return 0;
}

static struct bq27520_bat_platform_data bq27520_bat_pdata = {
    .i2c_bus_id      = BQ27520_I2C_BUS,
};

static struct i2c_board_info bq27520_bat_upt_i2c_board_info = {
    .type          = BQ27520_DEV_UPT_NAME,
    .flags         = 0x00,
    .addr          = BQ27520_I2C_DEFAULT_UPT_ADDR,
    .archdata      = NULL,
    .irq           = -1,
    .platform_data = &bq27520_bat_pdata,
};

static struct i2c_device_id bq27520_bat_upt_i2c_id[] = {
    { BQ27520_DEV_UPT_NAME, 0 },
    { },
};

static struct i2c_driver bq27520_bat_upt_i2c_driver = {
    .driver    = {
        .name  = BQ27520_DEV_UPT_NAME,
        .owner = THIS_MODULE,
    },
    .probe     = bq27520_bat_upt_i2c_probe,
    .id_table  = bq27520_bat_upt_i2c_id,
};

int bq27520_bat_upt_i2c_init(void)
{
    int ret = 0;
    struct i2c_adapter *adapter = NULL;
    struct i2c_client *client   = NULL;
    struct i2c_board_info *board_info= NULL;

    BAT_DBG("++++ %s ++++\n", __func__);

    //register i2c device
    board_info = &bq27520_bat_upt_i2c_board_info;
    adapter = i2c_get_adapter(((struct bq27520_bat_platform_data *)board_info->platform_data)->i2c_bus_id);
    if (adapter == NULL) {
        BAT_DBG_E("can not get i2c adapter, client address error\n");
        ret = -ENODEV;
        goto get_adapter_fail;
    }

    client = i2c_new_device(adapter, board_info);
    if (client == NULL) {
        BAT_DBG("allocate i2c client failed\n");
        ret = -ENOMEM;
        goto register_i2c_device_fail;
    }
    i2c_put_adapter(adapter);

    mutex_lock(&batt_dev_upt_mutex);
    batt_upt_dev_info.i2c = client;
    mutex_unlock(&batt_dev_upt_mutex);

    //register i2c driver
    ret =  i2c_add_driver(&bq27520_bat_upt_i2c_driver);
    if (ret) {
        BAT_DBG("register bq27520 battery update i2c driver failed\n");
        goto i2c_register_driver_fail;
    }

    return ret;

i2c_register_driver_fail:
    i2c_unregister_device(client);
register_i2c_device_fail:
get_adapter_fail:
    return ret;
}

void bq27520_bat_upt_i2c_exit(void)
{
    struct i2c_client *client = NULL;

    asus_battery_exit();

    mutex_lock(&batt_dev_upt_mutex);
    client = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);

    i2c_unregister_device(batt_upt_dev_info.i2c);
    i2c_del_driver(&bq27520_bat_upt_i2c_driver);

    BAT_DBG("%s exit\n", __func__);
}
