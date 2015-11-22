/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 */

#include "smb345_external_include.h"
#include "asus_battery.h"
#include <linux/usb/penwell_otg.h>
//#include "../../intel_mdf_charger.h"
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_gpadc.h>

static inline bool is_ttl_valid(u64 ttl)
{
    return time_before64(get_jiffies_64(), ttl);
}

void *adc_handle;
static unsigned long long adc_ttl;
static int adc_sensor_vals[MSIC_BATT_SENSORS];
static int multi_read_adc_regs(void *adc_handle,
            int sample_count, int channel_count, ...)
{
    va_list args;
    int ret = 0, i, sensor, tmp;
    int *adc_val;
    int temp_adc_val[MSIC_BATT_SENSORS];

    ret = intel_mid_gpadc_sample(adc_handle, sample_count,
                &adc_sensor_vals[MSIC_ADC_TEMP_IDX],
                &adc_sensor_vals[MSIC_ADC_USB_VOL_IDX],
                &adc_sensor_vals[MSIC_ADC_BATTID_IDX]);
    if (ret) {
        BAT_DBG_E(" adc driver api returned error(%d)\n", ret);
        goto adc_multi_exit;
    }

    memcpy(temp_adc_val, adc_sensor_vals, sizeof(temp_adc_val));

    va_start(args, channel_count);
    for (i = 0; i < channel_count; i++)
    {
        /* get sensor number */
        sensor = va_arg(args, int);

        /* get value pointer */
        adc_val = va_arg(args, int *);
        if (adc_val == NULL) {
            ret = -EINVAL;
            goto adc_multi_exit;
        }
        *adc_val = temp_adc_val[sensor];

        switch (sensor)
        {
        case MSIC_ADC_TEMP_IDX:
            BAT_DBG_E(" %s: MSIC_ADC_TEMP_IDX:", __func__);
            //tmp = adc_to_temp(*adc_val);
            break;
        case MSIC_ADC_USB_VOL_IDX:
            BAT_DBG_E(" %s: MSIC_ADC_USB_VOL_IDX:", __func__);
            tmp = MSIC_ADC_TO_VBUS_VOL(*adc_val);
            break;
        case MSIC_ADC_BATTID_IDX:
            BAT_DBG_E(" %s: MSIC_ADC_BATTID_IDX:", __func__);
            tmp = *adc_val;
            break;
        default:
            BAT_DBG_E(" invalid sensor%d\n", sensor);
            return -EINVAL;
        }
        *adc_val = tmp;
    }
    va_end(args);

adc_multi_exit:
    return ret;
}

static int read_adc_regs(int sensor, int *sensor_val, void *adc_handle)
{
    int ret;
    ret = multi_read_adc_regs(adc_handle, 1, 1, sensor, sensor_val);

    if (ret)
        BAT_DBG_E(" %s:multi_read_adc_regs failed", __func__);
    return ret;
}

int pmic_get_batt_id()
{
    int ret;
    int batt_id;

    if (!adc_handle)
        adc_handle = intel_mid_gpadc_alloc(MSIC_BATT_SENSORS,
            MSIC_BATT_PACK_TEMP | CH_NEED_VCALIB | CH_NEED_VREF,
            MSIC_USB_VOLTAGE | CH_NEED_VCALIB,
            MSIC_BATTID | CH_NEED_VREF | CH_NEED_VCALIB);

    if (!adc_handle)
        BAT_DBG_E(" %s: ADC allocation failed\n", __func__);

    ret = read_adc_regs(MSIC_ADC_BATTID_IDX, &batt_id, adc_handle);
    if (ret) {
        return ret;
        BAT_DBG_E(" %s: read ADC failed\n", __func__);
    }

    return batt_id;
}

#ifndef FE170CG_USER_BUILD
int pmic_dump_registers(int dump_mask, struct seq_file *s)
{
    int i, retval = 0;
    uint8_t regs_val;
    uint16_t chk_regs_addr;
    uint16_t regs_addr_boot[] = {
        MSIC_BATT_RESETIRQ1_ADDR, MSIC_BATT_RESETIRQ2_ADDR,
        MSIC_BATT_CHR_LOWBATTDET_ADDR, MSIC_BATT_CHR_SPCHARGER_ADDR,
        MSIC_BATT_CHR_CHRTTIME_ADDR, MSIC_BATT_CHR_CHRCTRL1_ADDR,
        MSIC_BATT_CHR_CHRSTWDT_ADDR, MSIC_BATT_CHR_CHRSAFELMT_ADDR
    };

    char *regs_str_boot[] = {
        "rirq1", "rirq2", "lowdet",
        "spchr", "chrtime", "chrctrl1",
        "chrgwdt", "safelmt"
    };

    uint16_t regs_addr_int[] = {
        MSIC_BATT_CHR_PWRSRCINT_ADDR, MSIC_BATT_CHR_PWRSRCINT1_ADDR,
        MSIC_BATT_CHR_CHRINT_ADDR, MSIC_BATT_CHR_CHRINT1_ADDR,
        MSIC_BATT_CHR_PWRSRCLMT_ADDR
    };

    char *regs_str_int[] = {
        "pwrint", "pwrint1", "chrint",
        "chrint1", "pwrsrclmt"
    };

    uint16_t regs_addr_evt[] = {
        MSIC_BATT_CHR_CHRCTRL_ADDR, MSIC_BATT_CHR_CHRCVOLTAGE_ADDR,
        MSIC_BATT_CHR_CHRCCURRENT_ADDR, MSIC_BATT_CHR_SPWRSRCINT_ADDR,
        MSIC_BATT_CHR_SPWRSRCINT1_ADDR, CHR_STATUS_FAULT_REG
    };

    char *regs_str_evt[] = {
        "chrctrl", "chrcv", "chrcc",
        "spwrsrcint", "sprwsrcint1", "chrflt"
    };

    uint16_t regs_addr_others[] = {
        MSIC_BATT_CHR_MPWRSRCINT_ADDR, MSIC_BATT_CHR_MPWRSRCINT1_ADDR,
        MSIC_BATT_CHR_MCHRINT_ADDR, MSIC_BATT_CHR_MCHRINT1_ADDR,
        MSIC_BATT_CHR_VBUSDET_ADDR, MSIC_BATT_CHR_WDTWRITE_ADDR
    };

    char *regs_str_others[] = {
        "chrmpwrsrcint", "chrmpwrsrcint1", "chrmchrint",
        "chrmchrint1", "chrvbusdet", "chrwdtwrite"
    };

    if (dump_mask & MSIC_CHRG_REG_DUMP_BOOT) {
        for (i = 0; i < ARRAY_SIZE(regs_addr_boot); i++) {
            retval = intel_scu_ipc_ioread8(regs_addr_boot[i], &regs_val);
            if (retval) {
                chk_regs_addr = regs_addr_boot[i];
                goto ipcread_err;
            }
            BAT_DBG(" PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN
                "\t%s\n", regs_addr_boot[i],
                BYTETOBINARY(regs_val), regs_str_boot[i]);
            if (s)
                seq_printf(s, "0x%03x:   " BYTETOBINARYPATTERN
                    "   %s\n", regs_addr_boot[i],
                    BYTETOBINARY(regs_val), regs_str_boot[i]);
        }
    }

    if (dump_mask & MSIC_CHRG_REG_DUMP_INT) {
        for (i = 0; i < ARRAY_SIZE(regs_addr_int); i++) {
            retval = intel_scu_ipc_ioread8(regs_addr_int[i], &regs_val);
            if (retval) {
                chk_regs_addr = regs_addr_int[i];
                goto ipcread_err;
            }
            BAT_DBG(" PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN
                "\t%s\n", regs_addr_int[i],
                BYTETOBINARY(regs_val), regs_str_int[i]);
            if (s)
                seq_printf(s, "0x%03x:   " BYTETOBINARYPATTERN
                    "   %s\n", regs_addr_int[i],
                    BYTETOBINARY(regs_val), regs_str_int[i]);
        }
    }

    if (dump_mask & MSIC_CHRG_REG_DUMP_EVENT) {
        for (i = 0; i < ARRAY_SIZE(regs_addr_evt); i++) {
            retval = intel_scu_ipc_ioread8(regs_addr_evt[i], &regs_val);
            if (retval) {
                chk_regs_addr = regs_addr_evt[i];
                goto ipcread_err;
            }
            BAT_DBG(" PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN
                "\t%s\n", regs_addr_evt[i],
                BYTETOBINARY(regs_val), regs_str_evt[i]);
            if (s)
                seq_printf(s, "0x%03x:   " BYTETOBINARYPATTERN
                    "   %s\n", regs_addr_evt[i],
                    BYTETOBINARY(regs_val), regs_str_evt[i]);
        }
    }

    if (dump_mask & MSIC_CHRG_REG_DUMP_OTHERS) {
        for (i = 0; i < ARRAY_SIZE(regs_addr_others); i++) {
            retval = intel_scu_ipc_ioread8(regs_addr_others[i], &regs_val);
            if (retval) {
                chk_regs_addr = regs_addr_others[i];
                goto ipcread_err;
            }
            BAT_DBG(" PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN
                "\t%s\n", regs_addr_others[i],
                BYTETOBINARY(regs_val), regs_str_others[i]);
            if (s)
                seq_printf(s, "0x%03x:   " BYTETOBINARYPATTERN
                    "   %s\n", regs_addr_others[i],
                    BYTETOBINARY(regs_val), regs_str_others[i]);
        }
    }

    if (s)
        seq_printf(s, "\n");

    return 0;

ipcread_err:
    BAT_DBG_E("%s: ipcread_err: address: 0x%03x!!!\n",
        __func__, chk_regs_addr);
    if (s)
        seq_printf(s, "%s: ipcread_err: address: 0x%03x!!!\n",
            __func__, chk_regs_addr);

    return 0;
}
#else
int pmic_dump_registers(int dump_mask, struct seq_file *s) { return 0; }
#endif

