/*
 * Support for Medfield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef _css_receiver_2400_defs_h_
#define _css_receiver_2400_defs_h_

#define CSS_RECEIVER_DATA_WIDTH                8
#define CSS_RECEIVER_RX_TRIG                   4
#define CSS_RECEIVER_RF_WORD                  32
#define CSS_RECEIVER_IMG_PROC_RF_ADDR         10
#define CSS_RECEIVER_CSI_RF_ADDR               4
#define CSS_RECEIVER_DATA_OUT                 12
#define CSS_RECEIVER_CHN_NO                    2
#define CSS_RECEIVER_DWORD_CNT                11
#define CSS_RECEIVER_FORMAT_TYP                5
#define CSS_RECEIVER_HRESPONSE                 2
#define CSS_RECEIVER_STATE_WIDTH               3
#define CSS_RECEIVER_FIFO_DAT                 32
#define CSS_RECEIVER_CNT_VAL                   2
#define CSS_RECEIVER_PRED10_VAL               10
#define CSS_RECEIVER_PRED12_VAL               12
#define CSS_RECEIVER_CNT_WIDTH                 8
#define CSS_RECEIVER_WORD_CNT                 16
#define CSS_RECEIVER_PIXEL_LEN                 6
#define CSS_RECEIVER_PIXEL_CNT                 5
#define CSS_RECEIVER_COMP_8_BIT                8
#define CSS_RECEIVER_COMP_7_BIT                7
#define CSS_RECEIVER_COMP_6_BIT                6
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_DATA_WIDTH     16
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_CH_ID_WIDTH     2
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_FMT_TYPE_WIDTH  3
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_REAL_WIDTH (_HRT_CSS_RECEIVER_2400_GEN_SHORT_DATA_WIDTH + _HRT_CSS_RECEIVER_2400_GEN_SHORT_CH_ID_WIDTH + _HRT_CSS_RECEIVER_2400_GEN_SHORT_FMT_TYPE_WIDTH)
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_WIDTH      32 /* use 32 to be compatibel with streaming monitor !, MSB's of interface are tied to '0' */ 

#define CSI_CONFIG_WIDTH                       4             

/* division of gen_short data, ch_id and fmt_type over streaming data interface */
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_DATA_BIT_LSB     0
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_FMT_TYPE_BIT_LSB (_HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_DATA_BIT_LSB     + _HRT_CSS_RECEIVER_2400_GEN_SHORT_DATA_WIDTH)
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_CH_ID_BIT_LSB    (_HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_FMT_TYPE_BIT_LSB + _HRT_CSS_RECEIVER_2400_GEN_SHORT_FMT_TYPE_WIDTH)
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_DATA_BIT_MSB     (_HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_FMT_TYPE_BIT_LSB - 1)
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_FMT_TYPE_BIT_MSB (_HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_CH_ID_BIT_LSB    - 1)
#define _HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_CH_ID_BIT_MSB    (_HRT_CSS_RECEIVER_2400_GEN_SHORT_STR_REAL_WIDTH       - 1)

#define _HRT_CSS_RECEIVER_2400_REG_ALIGN 4
#define _HRT_CSS_RECEIVER_2400_BYTES_PER_PKT             4

#define hrt_css_receiver_2400_4_lane_port_offset  0x100
#define hrt_css_receiver_2400_1_lane_port_offset  0x200
#define hrt_css_receiver_2400_2_lane_port_offset  0x300
#define hrt_css_receiver_2400_backend_port_offset 0x100

#define _HRT_CSS_RECEIVER_2400_DEVICE_READY_REG_IDX      0
#define _HRT_CSS_RECEIVER_2400_IRQ_STATUS_REG_IDX        1
#define _HRT_CSS_RECEIVER_2400_IRQ_ENABLE_REG_IDX        2
#define _HRT_CSS_RECEIVER_2400_CSI2_FUNC_PROG_REG_IDX    3
#define _HRT_CSS_RECEIVER_2400_INIT_COUNT_REG_IDX        4
#define _HRT_CSS_RECEIVER_2400_FS_TO_LS_DELAY_REG_IDX    7
#define _HRT_CSS_RECEIVER_2400_LS_TO_DATA_DELAY_REG_IDX  8
#define _HRT_CSS_RECEIVER_2400_DATA_TO_LE_DELAY_REG_IDX  9
#define _HRT_CSS_RECEIVER_2400_LE_TO_FE_DELAY_REG_IDX   10
#define _HRT_CSS_RECEIVER_2400_FE_TO_FS_DELAY_REG_IDX   11
#define _HRT_CSS_RECEIVER_2400_LE_TO_LS_DELAY_REG_IDX   12
#define _HRT_CSS_RECEIVER_2400_TWO_PIXEL_EN_REG_IDX     13
#define _HRT_CSS_RECEIVER_2400_RAW16_18_DATAID_REG_IDX  14
#define _HRT_CSS_RECEIVER_2400_SYNC_COUNT_REG_IDX       15
#define _HRT_CSS_RECEIVER_2400_RX_COUNT_REG_IDX         16
#define _HRT_CSS_RECEIVER_2400_BACKEND_RST_REG_IDX      17
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG0_IDX 18
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG1_IDX 19
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG0_IDX 20
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG1_IDX 21
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG0_IDX 22
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG1_IDX 23
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC3_REG0_IDX 24
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC3_REG1_IDX 25
#define _HRT_CSS_RECEIVER_2400_RAW18_REG_IDX            26
#define _HRT_CSS_RECEIVER_2400_FORCE_RAW8_REG_IDX       27
#define _HRT_CSS_RECEIVER_2400_RAW16_REG_IDX            28

/* Interrupt bits for IRQ_STATUS and IRQ_ENABLE registers */
#define _HRT_CSS_RECEIVER_2400_IRQ_OVERRUN_BIT                0
#define _HRT_CSS_RECEIVER_2400_IRQ_RESERVED_BIT               1
#define _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_ENTRY_BIT       2
#define _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_EXIT_BIT        3
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_HS_BIT             4
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_SYNC_HS_BIT        5
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_CONTROL_BIT            6
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_DOUBLE_BIT         7
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_CORRECTED_BIT      8
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_NO_CORRECTION_BIT  9
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_CRC_BIT               10
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ID_BIT                11
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_SYNC_BIT        12
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_DATA_BIT        13
#define _HRT_CSS_RECEIVER_2400_IRQ_DATA_TIMEOUT_BIT          14
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ESCAPE_BIT            15
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_LINE_SYNC_BIT         16

#define _HRT_CSS_RECEIVER_2400_IRQ_OVERRUN_CAUSE_                  "Fifo Overrun"
#define _HRT_CSS_RECEIVER_2400_IRQ_RESERVED_CAUSE_                 "Reserved"
#define _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_ENTRY_CAUSE_         "Sleep mode entry"
#define _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_EXIT_CAUSE_          "Sleep mode exit"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_HS_CAUSE_               "Error high speed SOT"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_SYNC_HS_CAUSE_          "Error high speed sync SOT"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_CONTROL_CAUSE_              "Error control"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_DOUBLE_CAUSE_           "Error correction double bit"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_CORRECTED_CAUSE_        "Error correction single bit"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_NO_CORRECTION_CAUSE_    "No error"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_CRC_CAUSE_                  "Error cyclic redundancy check"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ID_CAUSE_                   "Error id"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_SYNC_CAUSE_           "Error frame sync"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_DATA_CAUSE_           "Error frame data"
#define _HRT_CSS_RECEIVER_2400_IRQ_DATA_TIMEOUT_CAUSE_             "Data time-out"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_ESCAPE_CAUSE_               "Error escape"
#define _HRT_CSS_RECEIVER_2400_IRQ_ERR_LINE_SYNC_CAUSE_            "Error line sync"

/* Bits for CSI2_DEVICE_READY register */
#define _HRT_CSS_RECEIVER_2400_CSI2_DEVICE_READY_IDX                          0
#define _HRT_CSS_RECEIVER_2400_CSI2_MASK_INIT_TIME_OUT_ERR_IDX                2
#define _HRT_CSS_RECEIVER_2400_CSI2_MASK_OVER_RUN_ERR_IDX                     3
#define _HRT_CSS_RECEIVER_2400_CSI2_MASK_SOT_SYNC_ERR_IDX                     4
#define _HRT_CSS_RECEIVER_2400_CSI2_MASK_RECEIVE_DATA_TIME_OUT_ERR_IDX        5
#define _HRT_CSS_RECEIVER_2400_CSI2_MASK_ECC_TWO_BIT_ERR_IDX                  6
#define _HRT_CSS_RECEIVER_2400_CSI2_MASK_DATA_ID_ERR_IDX                      7

                                  
/* Bits for CSI2_FUNC_PROG register */
#define _HRT_CSS_RECEIVER_2400_CSI2_DATA_TIMEOUT_IDX    0
#define _HRT_CSS_RECEIVER_2400_CSI2_DATA_TIMEOUT_BITS   19

/* Bits for INIT_COUNT register */
#define _HRT_CSS_RECEIVER_2400_INIT_TIMER_IDX  0
#define _HRT_CSS_RECEIVER_2400_INIT_TIMER_BITS 16

/* Bits for COUNT registers */
#define _HRT_CSS_RECEIVER_2400_SYNC_COUNT_IDX     0
#define _HRT_CSS_RECEIVER_2400_SYNC_COUNT_BITS    8
#define _HRT_CSS_RECEIVER_2400_RX_COUNT_IDX       0
#define _HRT_CSS_RECEIVER_2400_RX_COUNT_BITS      8

/* Bits for RAW116_18_DATAID register */
#define _HRT_CSS_RECEIVER_2400_RAW16_18_DATAID_RAW16_BITS_IDX   0
#define _HRT_CSS_RECEIVER_2400_RAW16_18_DATAID_RAW16_BITS_BITS  6
#define _HRT_CSS_RECEIVER_2400_RAW16_18_DATAID_RAW18_BITS_IDX   8
#define _HRT_CSS_RECEIVER_2400_RAW16_18_DATAID_RAW18_BITS_BITS  6

/* Bits for COMP_FORMAT register, this selects the compression data format */
#define _HRT_CSS_RECEIVER_2400_COMP_RAW_BITS_IDX  0
#define _HRT_CSS_RECEIVER_2400_COMP_RAW_BITS_BITS 8
#define _HRT_CSS_RECEIVER_2400_COMP_NUM_BITS_IDX  (_HRT_CSS_RECEIVER_2400_COMP_RAW_BITS_IDX + _HRT_CSS_RECEIVER_2400_COMP_RAW_BITS_BITS)
#define _HRT_CSS_RECEIVER_2400_COMP_NUM_BITS_BITS 8

/* Bits for COMP_PREDICT register, this selects the predictor algorithm */
#define _HRT_CSS_RECEIVER_2400_PREDICT_NO_COMP 0
#define _HRT_CSS_RECEIVER_2400_PREDICT_1       1
#define _HRT_CSS_RECEIVER_2400_PREDICT_2       2

/* Number of bits used for the delay registers */
#define _HRT_CSS_RECEIVER_2400_DELAY_BITS 8

/* Bits for COMP_SCHEME register, this  selects the compression scheme for a VC */
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD1_BITS_IDX  0
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD2_BITS_IDX  5
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD3_BITS_IDX  10
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD4_BITS_IDX  15
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD5_BITS_IDX  20
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD6_BITS_IDX  25
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD7_BITS_IDX  0
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD8_BITS_IDX  5
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD_BITS_BITS  5
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD_FMT_BITS_IDX   0
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD_FMT_BITS_BITS  3
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD_PRED_BITS_IDX  3
#define _HRT_CSS_RECEIVER_2400_COMP_SCHEME_USD_PRED_BITS_BITS 2


/* BITS for backend RAW16 and RAW 18 registers */


#define _HRT_CSS_RECEIVER_2400_RAW18_DATAID_IDX    0
#define _HRT_CSS_RECEIVER_2400_RAW18_DATAID_BITS   6
#define _HRT_CSS_RECEIVER_2400_RAW18_OPTION_IDX    6
#define _HRT_CSS_RECEIVER_2400_RAW18_OPTION_BITS   2
#define _HRT_CSS_RECEIVER_2400_RAW18_EN_IDX        8
#define _HRT_CSS_RECEIVER_2400_RAW18_EN_BITS       1

#define _HRT_CSS_RECEIVER_2400_RAW16_DATAID_IDX    0
#define _HRT_CSS_RECEIVER_2400_RAW16_DATAID_BITS   6
#define _HRT_CSS_RECEIVER_2400_RAW16_OPTION_IDX    6
#define _HRT_CSS_RECEIVER_2400_RAW16_OPTION_BITS   2
#define _HRT_CSS_RECEIVER_2400_RAW16_EN_IDX        8
#define _HRT_CSS_RECEIVER_2400_RAW16_EN_BITS       1

/* These hsync and vsync values are for HSS simulation only */
#define _HRT_CSS_RECEIVER_2400_HSYNC_VAL (1<<16)
#define _HRT_CSS_RECEIVER_2400_VSYNC_VAL (1<<17)

/* Definition of data format ID at the interface CSS_receiver capture/acquisition units */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV420_8          24   /* 01 1000 YUV420 8-bit                                        */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV420_10         25   /* 01 1001  YUV420 10-bit                                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV420_8L         26   /* 01 1010   YUV420 8-bit legacy                               */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV422_8          30   /* 01 1110   YUV422 8-bit                                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV422_10         31   /* 01 1111   YUV422 10-bit                                     */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RGB444            32   /* 10 0000   RGB444                                            */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RGB555            33   /* 10 0001   RGB555                                            */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RGB565            34   /* 10 0010   RGB565                                            */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RGB666            35   /* 10 0011   RGB666                                            */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RGB888            36   /* 10 0100   RGB888                                            */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RAW6              40   /* 10 1000   RAW6                                              */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RAW7              41   /* 10 1001   RAW7                                              */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RAW8              42   /* 10 1010   RAW8                                              */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RAW10             43   /* 10 1011   RAW10                                             */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RAW12             44   /* 10 1100   RAW12                                             */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_RAW14             45   /* 10 1101   RAW14                                             */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_1         48   /* 11 0000    JPEG [User Defined 8-bit Data Type 1]            */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_2         49   /* 11 0001    User Defined 8-bit Data Type 2                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_3         50   /* 11 0010    User Defined 8-bit Data Type 3                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_4         51   /* 11 0011    User Defined 8-bit Data Type 4                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_5         52   /* 11 0100    User Defined 8-bit Data Type 5                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_6         53   /* 11 0101    User Defined 8-bit Data Type 6                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_7         54   /* 11 0110    User Defined 8-bit Data Type 7                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_USR_DEF_8         55   /* 11 0111    User Defined 8-bit Data Type 8                   */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_Emb               18   /* 01 0010    embedded eight bit non image data                */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_SOF                0   /* 00 0000    frame start                                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_EOF                1   /* 00 0001    frame end                                        */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_SOL                2   /* 00 0010    line start                                       */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_EOL                3   /* 00 0011    line end                                         */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH1            8   /* 00 1000  Generic Short Packet Code 1                        */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH2            9   /* 00 1001    Generic Short Packet Code 2                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH3           10   /* 00 1010    Generic Short Packet Code 3                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH4           11   /* 00 1011    Generic Short Packet Code 4                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH5           12   /* 00 1100    Generic Short Packet Code 5                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH6           13   /* 00 1101    Generic Short Packet Code 6                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH7           14   /* 00 1110    Generic Short Packet Code 7                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_GEN_SH8           15   /* 00 1111    Generic Short Packet Code 8                      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV420_8_CSPS     28   /* 01 1100   YUV420 8-bit (Chroma Shifted Pixel Sampling)      */
#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_YUV420_10_CSPS    29   /* 01 1101   YUV420 10-bit (Chroma Shifted Pixel Sampling)     */

#define _HRT_CSS_RECEIVER_2400_DATA_FORMAT_ID_WIDTH              6

/* Definition of format_types at the interface CSS --> input_selector*/
/* !! Changes here should be copied to systems/isp/isp_css/bin/conv_transmitter_cmd.tcl !! */
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RGB888           0  // 36 'h24
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RGB555           1  // 33 'h
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RGB444           2  // 32
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RGB565           3  // 34
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RGB666           4  // 35
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW8             5  // 42 
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW10            6  // 43
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW6             7  // 40
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW7             8  // 41
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW12            9  // 43
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW14           10  // 45
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV420_8        11  // 30
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV420_10       12  // 25
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV422_8        13  // 30
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV422_10       14  // 31
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_1       15  // 48
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV420_8L       16  // 26
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_Emb             17  // 18
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_2       18  // 49
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_3       19  // 50
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_4       20  // 51
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_5       21  // 52
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_6       22  // 53
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_7       23  // 54
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_USR_DEF_8       24  // 55
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV420_8_CSPS   25  // 28
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_YUV420_10_CSPS  26  // 29
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW16           27  // ?
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW18           28  // ?
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW18_2         29  // ? Option 2 for depacketiser
#define _HRT_CSS_RECEIVER_2400_FMT_TYPE_RAW18_3         30  // ? Option 3 for depacketiser 

/* packet bit definition */
#define _HRT_CSS_RECEIVER_2400_PKT_SOP_IDX                        32
#define _HRT_CSS_RECEIVER_2400_PKT_SOP_BITS                        1
#define _HRT_CSS_RECEIVER_2400_PKT_CH_ID_IDX                      22
#define _HRT_CSS_RECEIVER_2400_PKT_CH_ID_BITS                      2
#define _HRT_CSS_RECEIVER_2400_PKT_FMT_ID_IDX                     16
#define _HRT_CSS_RECEIVER_2400_PKT_FMT_ID_BITS                     6
#define _HRT_CSS_RECEIVER_2400_PH_DATA_FIELD_IDX                   0
#define _HRT_CSS_RECEIVER_2400_PH_DATA_FIELD_BITS                 16
#define _HRT_CSS_RECEIVER_2400_PKT_PAYLOAD_IDX                     0
#define _HRT_CSS_RECEIVER_2400_PKT_PAYLOAD_BITS                   32

/* definition for state machine of data FIFO for decode different type of data */

#define _HRT_CSS_RECEIVER_2400_YUV420_8_REPEAT_PTN                 1  
#define _HRT_CSS_RECEIVER_2400_YUV420_10_REPEAT_PTN                5
#define _HRT_CSS_RECEIVER_2400_YUV420_8L_REPEAT_PTN                1
#define _HRT_CSS_RECEIVER_2400_YUV422_8_REPEAT_PTN                 1
#define _HRT_CSS_RECEIVER_2400_YUV422_10_REPEAT_PTN                5
#define _HRT_CSS_RECEIVER_2400_RGB444_REPEAT_PTN                   2 
#define _HRT_CSS_RECEIVER_2400_RGB555_REPEAT_PTN                   2
#define _HRT_CSS_RECEIVER_2400_RGB565_REPEAT_PTN                   2
#define _HRT_CSS_RECEIVER_2400_RGB666_REPEAT_PTN                   9                       
#define _HRT_CSS_RECEIVER_2400_RGB888_REPEAT_PTN                   3
#define _HRT_CSS_RECEIVER_2400_RAW6_REPEAT_PTN                     3
#define _HRT_CSS_RECEIVER_2400_RAW7_REPEAT_PTN                     7
#define _HRT_CSS_RECEIVER_2400_RAW8_REPEAT_PTN                     1
#define _HRT_CSS_RECEIVER_2400_RAW10_REPEAT_PTN                    5
#define _HRT_CSS_RECEIVER_2400_RAW12_REPEAT_PTN                    3        
#define _HRT_CSS_RECEIVER_2400_RAW14_REPEAT_PTN                    7
//#define CSS_RECEIVER_USR_DEF_1_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_2_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_3_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_4_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_5_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_6_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_7_REPEAT_PTN     
//#define CSS_RECEIVER_USR_DEF_8_REPEAT_PTN     
//#define CSS_RECEIVER_Emb_REPEAT_PTN                  
//#define CSS_RECEIVER_YUV420_8_CSPS_REPEAT_PTN 
//#define CSS_RECEIVER_YUV420_10_CSPS_REPEAT_PTN

#define _HRT_CSS_RECEIVER_2400_MAX_REPEAT_PTN                      _HRT_CSS_RECEIVER_2400_RGB666_REPEAT_PTN

// SH Backend Register IDs
#define _HRT_CSS_RECEIVER_2400_BE_GSP_ACC_OVL_REG_IDX              0
#define _HRT_CSS_RECEIVER_2400_BE_SRST_REG_IDX                     1
#define _HRT_CSS_RECEIVER_2400_BE_TWO_PPC_REG_IDX                  2
#define _HRT_CSS_RECEIVER_2400_BE_COMP_FORMAT_REG0_IDX             3
#define _HRT_CSS_RECEIVER_2400_BE_COMP_FORMAT_REG1_IDX             4
#define _HRT_CSS_RECEIVER_2400_BE_COMP_FORMAT_REG2_IDX             5
#define _HRT_CSS_RECEIVER_2400_BE_COMP_FORMAT_REG3_IDX             6
#define _HRT_CSS_RECEIVER_2400_BE_SEL_REG_IDX                      7
#define _HRT_CSS_RECEIVER_2400_BE_RAW16_CONFIG_REG_IDX             8
#define _HRT_CSS_RECEIVER_2400_BE_RAW18_CONFIG_REG_IDX             9
#define _HRT_CSS_RECEIVER_2400_BE_FORCE_RAW8_REG_IDX              10
#define _HRT_CSS_RECEIVER_2400_BE_IRQ_STATUS_REG_IDX              11
#define _HRT_CSS_RECEIVER_2400_BE_IRQ_CLEAR_REG_IDX               12

#define _HRT_CSS_RECEIVER_2400_BE_NOF_REGISTERS                   13

#define _HRT_CSS_RECEIVER_2400_BE_COMP_FMT_IDX                     0
#define _HRT_CSS_RECEIVER_2400_BE_COMP_FMT_WIDTH                   3
#define _HRT_CSS_RECEIVER_2400_BE_COMP_PRED_IDX                    3
#define _HRT_CSS_RECEIVER_2400_BE_COMP_PRED_WIDTH                  1
#define _HRT_CSS_RECEIVER_2400_BE_COMP_USD_BITS                    4  /* bits per USD type */

#define _HRT_CSS_RECEIVER_2400_BE_RAW16_DATAID_IDX                 0
#define _HRT_CSS_RECEIVER_2400_BE_RAW16_EN_IDX                     6
#define _HRT_CSS_RECEIVER_2400_BE_RAW18_DATAID_IDX                 0
#define _HRT_CSS_RECEIVER_2400_BE_RAW18_OPTION_IDX                 6
#define _HRT_CSS_RECEIVER_2400_BE_RAW18_EN_IDX                     8

#define _HRT_CSS_RECEIVER_2400_BE_COMP_NO_COMP                     0
#define _HRT_CSS_RECEIVER_2400_BE_COMP_10_6_10                     1
#define _HRT_CSS_RECEIVER_2400_BE_COMP_10_7_10                     2
#define _HRT_CSS_RECEIVER_2400_BE_COMP_10_8_10                     3
#define _HRT_CSS_RECEIVER_2400_BE_COMP_12_6_12                     4
#define _HRT_CSS_RECEIVER_2400_BE_COMP_12_7_12                     5
#define _HRT_CSS_RECEIVER_2400_BE_COMP_12_8_12                     6

#define _HRT_CSS_RECEIVER_2400_BE_SRST_HE                          0
#define _HRT_CSS_RECEIVER_2400_BE_SRST_RCF                         1
#define _HRT_CSS_RECEIVER_2400_BE_SRST_PF                          2
#define _HRT_CSS_RECEIVER_2400_BE_SRST_SM                          3
#define _HRT_CSS_RECEIVER_2400_BE_SRST_PD                          4
#define _HRT_CSS_RECEIVER_2400_BE_SRST_SD                          5
#define _HRT_CSS_RECEIVER_2400_BE_SRST_OT                          6
#define _HRT_CSS_RECEIVER_2400_BE_SRST_BC                          7
#define _HRT_CSS_RECEIVER_2400_BE_SRST_WIDTH                       8

#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_WIDTH                 28
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_A_LSB              0
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_A_MSB             (_HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_A_LSB + CSS_RECEIVER_DATA_OUT - 1)
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_A_VAL_BIT         (_HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_A_MSB + 1)
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_B_LSB             (_HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_A_VAL_BIT + 1)
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_B_MSB             (_HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_B_LSB + CSS_RECEIVER_DATA_OUT - 1)
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_B_VAL_BIT         (_HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_B_MSB + 1)
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_SOP_BIT               (_HRT_CSS_RECEVIER_2400_BE_STREAMING_PIX_B_VAL_BIT + 1)
#define _HRT_CSS_RECEVIER_2400_BE_STREAMING_EOP_BIT               (_HRT_CSS_RECEVIER_2400_BE_STREAMING_SOP_BIT + 1)

#endif /* _css_receiver_2400_defs_h_ */
